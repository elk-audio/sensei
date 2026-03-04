/*
 * Copyright 2017-2026 Elk Audio AB
 *
 * SENSEI is free software: you can redistribute it and/or modify it under the terms of
 * the GNU Affero General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * SENSEI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License along with
 * SENSEI.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @brief gRPC-based user frontend with async API
 * @copyright 2017-2026 Elk Audio AB, Stockholm
 */
#include "grpc_user_frontend.h"
#include "async_call_handlers.h"
#include "logging.h"
#include "message/command_defs.h"

using namespace sensei;
using namespace sensei::user_frontend;

namespace {

SENSEI_GET_LOGGER_WITH_MODULE_NAME("grpc_user_frontend");

constexpr int   DEFAULT_GRPC_PORT = 50051;
constexpr auto* DEFAULT_GRPC_ADDRESS = "0.0.0.0";

} // anonymous namespace

//==============================================================================
// AsyncSenseiControllerImpl Implementation
//==============================================================================

AsyncSenseiControllerImpl::AsyncSenseiControllerImpl(GrpcUserFrontend* frontend)
    : _frontend(frontend),
      _running(false)
{
}

AsyncSenseiControllerImpl::~AsyncSenseiControllerImpl()
{
    shutdown();
}

void AsyncSenseiControllerImpl::start(const std::string& server_address)
{
    if (_running)
    {
        SENSEI_LOG_WARNING("Async service already running");
        return;
    }

    grpc::ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());

    _async_service = std::make_unique<sensei_rpc::SenseiController::AsyncService>();
    builder.RegisterService(_async_service.get());

    _cq = builder.AddCompletionQueue();

    _server = builder.BuildAndStart();
    if (!_server)
    {
        SENSEI_LOG_ERROR("Failed to start gRPC async server on {}", server_address);
        return;
    }

    SENSEI_LOG_INFO("gRPC async server listening on {}", server_address);

    _broadcast_manager = std::make_unique<EventBroadcastManager>();

    // Spawn initial call handlers (one for each RPC call)
    new SubscribeCallData(_async_service.get(), _cq.get(), _broadcast_manager.get());
    new UpdateLedCallData(_async_service.get(), _cq.get(), _frontend);
    new RefreshAllStatesCallData(_async_service.get(), _cq.get(), _frontend);
    new GetControllerMapCallData(_async_service.get(), _cq.get(), _frontend);

    _running = true;
    _cq_thread = std::thread(&AsyncSenseiControllerImpl::_handle_rpcs, this);

    SENSEI_LOG_INFO("Spawned completion queue worker thread");
}

void AsyncSenseiControllerImpl::shutdown()
{
    if (!_running)
    {
        return;
    }

    SENSEI_LOG_INFO("Shutting down async gRPC service");

    _running = false;
    _broadcast_manager->shutdown();
    _server->Shutdown();
    _cq->Shutdown();
    if (_cq_thread.joinable())
    {
        _cq_thread.join();
    }

    // drain the queue, ensure that memory is cleaned up in DONE state
    void* tag;
    bool  ok;
    while (_cq->Next(&tag, &ok))
    {
        auto* call_data = static_cast<CallDataBase*>(tag);
        call_data->stop();
        call_data->proceed();
    }

    SENSEI_LOG_INFO("Async gRPC service shutdown complete");
}

void AsyncSenseiControllerImpl::broadcast_event(const sensei_rpc::Event& event)
{
    if (_broadcast_manager && _running)
    {
        _broadcast_manager->broadcast_event(event);
    }
}

void AsyncSenseiControllerImpl::_handle_rpcs()
{
    void* tag;
    bool  ok;

    SENSEI_LOG_INFO("Completion queue worker thread started");

    // Process completion queue events
    while (_running && _cq->Next(&tag, &ok))
    {
        auto* call_data = static_cast<CallDataBase*>(tag);
        if (!ok)
        {
            call_data->stop();
        }
        call_data->proceed();
    }

    SENSEI_LOG_INFO("Completion queue worker thread exiting");
}

//==============================================================================
// GrpcUserFrontend Implementation
//==============================================================================

GrpcUserFrontend::GrpcUserFrontend(MessageHandler* handler,
                                   const int       max_n_sensors,
                                   ThreadingMode   threading_mode)
    : UserFrontend(handler, max_n_sensors, threading_mode),
      _listen_address(DEFAULT_GRPC_ADDRESS),
      _listen_port(DEFAULT_GRPC_PORT),
      _server_running(false)
{
    SENSEI_LOG_INFO("GrpcUserFrontend created with async API");
    {
        std::unique_lock<std::mutex> lock(_controller_map_mutex);
        _controller_map.resize(max_n_sensors);
    }
    _start_server();
}

GrpcUserFrontend::~GrpcUserFrontend()
{
    _stop_server();
}

void GrpcUserFrontend::_start_server()
{
    std::lock_guard<std::mutex> lock(_server_mutex);
    if (_server_running)
    {
        SENSEI_LOG_WARNING("gRPC server already running");
        return;
    }

    // Create async service implementation
    _service_impl = std::make_unique<AsyncSenseiControllerImpl>(this);

    // Start the async service
    std::string server_address = _listen_address + ":" + std::to_string(_listen_port);
    _service_impl->start(server_address);

    _server_running = true;

    SENSEI_LOG_INFO("gRPC async user frontend server started on {}", server_address);
}

void GrpcUserFrontend::_stop_server()
{
    std::lock_guard<std::mutex> lock(_server_mutex);

    if (!_server_running)
    {
        return;
    }

    SENSEI_LOG_INFO("Stopping gRPC async user frontend server");

    if (_service_impl)
    {
        _service_impl->shutdown();
    }

    _server_running = false;
    _service_impl.reset();

    SENSEI_LOG_INFO("gRPC async user frontend server stopped");
}

CommandErrorCode GrpcUserFrontend::apply_command(const Command* cmd)
{
    CommandErrorCode status = CommandErrorCode::OK;

    switch (cmd->type())
    {
        case CommandType::SET_GRPC_LISTEN_ADDRESS:
        {
            const auto typed_cmd = static_cast<const SetGrpcListenAddressCommand*>(cmd);
            _listen_address = typed_cmd->data();
            _stop_server();
            _start_server();
            SENSEI_LOG_INFO("gRPC listen address set to: {}", _listen_address);
        }
        break;

        case CommandType::SET_GRPC_LISTEN_PORT:
        {
            const auto typed_cmd = static_cast<const SetGrpcListenPortCommand*>(cmd);
            if ((_listen_port < 1000) || (_listen_port > 65535))
            {
                status = CommandErrorCode::INVALID_PORT_NUMBER;
                SENSEI_LOG_ERROR("Invalid gRPC port number: {}", _listen_port);
            }
            else
            {
                _listen_port = typed_cmd->data();
                _stop_server();
                _start_server();
                SENSEI_LOG_INFO("gRPC listen port set to: {}", _listen_port);
            }
        }
        break;

        default:
            status = CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE;
            break;
    }

    // If command was not handled, try in the parent class
    if (status == CommandErrorCode::UNHANDLED_COMMAND_FOR_SENSOR_TYPE)
    {
        return UserFrontend::apply_command(cmd);
    }
    else
    {
        return status;
    }
}

void GrpcUserFrontend::broadcast_event(const sensei_rpc::Event& event)
{
    if (_service_impl)
    {
        _service_impl->broadcast_event(event);
    }
}

int GrpcUserFrontend::num_subscribers() const
{
    return _service_impl->broadcast_manager()->num_subscribers();
}

void GrpcUserFrontend::update_controller(int controller_id, const std::string& name, SensorType type)
{
    std::unique_lock<std::mutex> lock(_controller_map_mutex);
    _controller_map[controller_id] = {name, type};
}

void GrpcUserFrontend::populate_controller_map(sensei_rpc::GetControllerMapResponse* response)
{
    SENSEI_LOG_INFO("GetControllerMap called");

    response->clear_pots();
    response->clear_switches();
    response->clear_encoders();
    response->clear_rotaries();
    response->clear_leds();

    std::unique_lock<std::mutex> lock(_controller_map_mutex);
    for (size_t i = 0; i < _controller_map.size(); ++i)
    {
        int         controller_id = static_cast<int>(i);
        const auto& name = _controller_map[i].name;
        auto        type = _controller_map[i].type;
        if (name.empty() || type == SensorType::UNDEFINED)
        {
            continue;
        }
        switch (type)
        {
            case SensorType::ANALOG_INPUT:
            {
                auto controller = response->add_pots();
                controller->set_id(controller_id);
                controller->set_name(name);
                break;
            }

            case SensorType::DIGITAL_INPUT:
            {
                auto controller = response->add_switches();
                controller->set_id(controller_id);
                controller->set_name(name);
                break;
            }

            case SensorType::DIGITAL_OUTPUT:
            {
                auto controller = response->add_leds();
                controller->set_id(controller_id);
                controller->set_name(name);
                break;
            }

            case SensorType::DISCRETE_INPUT:
            case SensorType::RANGE_INPUT:
            {
                auto controller = response->add_rotaries();
                controller->set_id(controller_id);
                controller->set_name(name);
                break;
            }

            case SensorType::RELATIVE_INPUT:
            {
                auto controller = response->add_encoders();
                controller->set_id(controller_id);
                controller->set_name(name);
                break;
            }

            default:
                SENSEI_LOG_ERROR("Unexpected sensor type ({}) for id={}", static_cast<int>(type), controller_id);
                break;
        }
    }
}

void GrpcUserFrontend::refresh_controller_values()
{
    std::unique_lock<std::mutex> lock(_controller_map_mutex);
    for (size_t i = 0; i < _controller_map.size(); ++i)
    {
        const auto& name = _controller_map[i].name;
        auto        type = _controller_map[i].type;
        if (!name.empty() && type != SensorType::UNDEFINED)
        {
            int controller_id = static_cast<int>(i);
            _handler->post_event(_factory.make_clear_previous_value_command(controller_id));
            _handler->post_event(_factory.make_get_value_command(controller_id));
        }
    }
}
