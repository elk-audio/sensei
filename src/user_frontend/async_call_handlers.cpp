/*
 * Copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk
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
 * @brief Async gRPC call handlers implementation
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#include "async_call_handlers.h"
#include "grpc_user_frontend.h"
#include "logging.h"

namespace sensei {
namespace user_frontend {

namespace {
SENSEI_GET_LOGGER_WITH_MODULE_NAME("async_call_handlers");
}

//==============================================================================
// SubscribeCallData Implementation
//==============================================================================

SubscribeCallData::SubscribeCallData(
    sensei_rpc::PinProxyService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    EventBroadcastManager* broadcast_mgr)
    : CallDataBase(service, cq),
      _responder(&_ctx),
      _broadcast_mgr(broadcast_mgr),
      _in_processing(false),
      _is_writing(false)
{
    proceed();
}

void SubscribeCallData::proceed()
{
    if (_state == State::CREATE)
    {
        _state = State::PROCESSING;

        // When a client calls SubscribeToEvents, this handler will be activated
        SENSEI_LOG_INFO("Subscribing to events");
        _service->RequestSubscribeToEvents(&_ctx, &_request, &_responder, _cq, _cq, this);
    }
    else if (_state == State::PROCESSING)
    {
        // if we're getting a new process() call then we can't still be writing
        _is_writing = false;

        // First time in PROCESSING: spawn new handler for next client
        if (!_in_processing)
        {
            new SubscribeCallData(_service, _cq, _broadcast_mgr);

            const auto &ids = _request.controller_ids();
            auto controller_ids = std::vector<int>(ids.begin(), ids.end());
            _broadcast_mgr->register_subscriber(this, std::move(controller_ids));

            _in_processing = true;
        }

        std::lock_guard<std::mutex> lock(_write_mutex);
        _start_write();
    }
    else
    {
        assert(_state == State::DONE);
        delete this;
    }
}

void SubscribeCallData::stop() {
    _broadcast_mgr->unregister_subscriber(this);
    _state = State::DONE;
}

void SubscribeCallData::enqueue_event(const sensei_rpc::Event& event)
{
    if (_state != State::PROCESSING)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(_write_mutex);
    _pending_events.push(event);
    _start_write();
}

// Must be called with _write_mutex held
void SubscribeCallData::_start_write()
{
    // if we haven't returned from our last Write yet then don't start another, can happen
    // with multiple enqueue_event() in quick succession
    if (_pending_events.empty() || _is_writing)
    {
        return;
    }

    _current_event = std::move(_pending_events.front());
    _pending_events.pop();

    // Start async write - completion will trigger proceed()
    _is_writing = true;
    _responder.Write(_current_event, this);
}

//==============================================================================
// UpdateLedCallData Implementation
//==============================================================================

UpdateLedCallData::UpdateLedCallData(
    sensei_rpc::PinProxyService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    GrpcUserFrontend* frontend)
    : CallDataBase(service, cq),
      _responder(&_ctx),
      _frontend(frontend)
{
    proceed();
}

void UpdateLedCallData::proceed()
{
    if (_state == State::CREATE)
    {
        _state = State::PROCESSING;
        _service->RequestUpdateLed(&_ctx, &_request, &_responder, _cq, _cq, this);
    }
    else if (_state == State::PROCESSING)
    {
        new UpdateLedCallData(_service, _cq, _frontend);

        int controller_id = _request.controller_id();
        bool active = _request.active();

        SENSEI_LOG_INFO("UpdateLed: controller_id={}, active={}", controller_id, active);

        _frontend->set_digital_output(controller_id, active);

        _state = State::DONE;
        _responder.Finish(_response, grpc::Status::OK, this);
    }
    else
    {
        delete this;
    }
}

//==============================================================================
// RefreshAllStatesCallData Implementation
//==============================================================================

RefreshAllStatesCallData::RefreshAllStatesCallData(
    sensei_rpc::PinProxyService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    GrpcUserFrontend *frontend)
    : CallDataBase(service, cq),
      _responder(&_ctx),
      _frontend(frontend)
{
    proceed();
}

void RefreshAllStatesCallData::proceed()
{
    if (_state == State::CREATE)
    {
        _state = State::PROCESSING;
        _service->RequestRefreshAllStates(&_ctx, &_request, &_responder, _cq, _cq, this);
    }
    else if (_state == State::PROCESSING)
    {
        new RefreshAllStatesCallData(_service, _cq, _frontend);

        _frontend->refresh_controller_values();

        _state = State::DONE;
        _responder.Finish(_response, grpc::Status::OK, this);
    }
    else
    {
        assert(_state == State::DONE);
        delete this;
    }
}

//==============================================================================
// GetControllerMapCallData Implementation
//==============================================================================

GetControllerMapCallData::GetControllerMapCallData(
    sensei_rpc::PinProxyService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    GrpcUserFrontend* frontend)
    : CallDataBase(service, cq),
      _responder(&_ctx),
      _frontend(frontend)
{
    proceed();
}

void GetControllerMapCallData::proceed()
{
    if (_state == State::CREATE)
    {
        _state = State::PROCESSING;
        _service->RequestGetControllerMap(&_ctx, &_request, &_responder, _cq, _cq, this);
    }
    else if (_state == State::PROCESSING)
    {
        new GetControllerMapCallData(_service, _cq, _frontend);

        _frontend->populate_controller_map(&_response);

        _state = State::DONE;
        _responder.Finish(_response, grpc::Status::OK, this);
    }
    else
    {
        assert(_state == State::DONE);
        delete this;
    }
}

void EventBroadcastManager::register_subscriber(SubscribeCallData* subscriber, std::vector<int> &&controller_ids)
{
    if (_shutting_down)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(_subscribers_mutex);
    _subscribers.push_back({subscriber, std::move(controller_ids)});
    SENSEI_LOG_INFO("Subscriber registered, total subscribers: {}", _subscribers.size());
}

void EventBroadcastManager::unregister_subscriber(SubscribeCallData* subscriber)
{
    std::lock_guard<std::mutex> lock(_subscribers_mutex);
    auto it = std::find_if(_subscribers.begin(), _subscribers.end(), 
        [&subscriber](const SubscriberData& s) { return s.subscriber == subscriber; });

    if (it != _subscribers.end())
    {
        _subscribers.erase(it);
        SENSEI_LOG_INFO("Subscriber unregistered, total subscribers: {}", _subscribers.size());
    }
}

void EventBroadcastManager::broadcast_event(const sensei_rpc::Event& event)
{
    if (_shutting_down)
    {
        return;
    }

    // Broadcast to all active subscribers
    std::lock_guard<std::mutex> lock(_subscribers_mutex);
    for (auto& data : _subscribers)
    {
        auto &ids = data.controller_ids;
        if (data.controller_ids.empty() ||
            std::find(ids.begin(), ids.end(), event.controller_id()) != ids.end())
        {
            data.subscriber->enqueue_event(event);
        }
    }
}

void EventBroadcastManager::shutdown()
{
    _shutting_down = true;
    SENSEI_LOG_INFO("EventBroadcastManager shutting down");
}

int EventBroadcastManager::num_subscribers() {
    std::lock_guard<std::mutex> lock(_subscribers_mutex);
    return _subscribers.size();
}

} // namespace user_frontend
} // namespace sensei
