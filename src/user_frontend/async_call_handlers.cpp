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
    pin_proxy::PinProxyService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    EventBroadcastManager* broadcast_mgr,
    GrpcUserFrontend* frontend)
    : _state(CREATE),
      _responder(&_ctx),
      _service(service),
      _cq(cq),
      _broadcast_mgr(broadcast_mgr),
      _frontend(frontend),
      _in_processing(false)
{
    proceed();
}

void SubscribeCallData::proceed()
{
    if (_state == CREATE)
    {
        _state = PROCESSING;

        // When a client calls SubscribeToEvents, this handler will be activated
        SENSEI_LOG_INFO("Subscribing to events");
        _service->RequestSubscribeToEvents(&_ctx, &_request, &_responder, _cq, _cq, this);
    }
    else if (_state == PROCESSING)
    {
        // First time in PROCESSING: spawn new handler for next client
        if (!_in_processing)
        {
            new SubscribeCallData(_service, _cq, _broadcast_mgr, _frontend);
            _broadcast_mgr->register_subscriber(this);
            _in_processing = true;
        }

        // This proceed() was called because a Write completed
        std::lock_guard<std::mutex> lock(_write_mutex);
        _start_write();
    }
    else
    {
        assert(_state == DONE);
        delete this;
    }
}

void SubscribeCallData::stop() {
    _broadcast_mgr->unregister_subscriber(this);
    _state = DONE;
}

void SubscribeCallData::enqueue_event(const pin_proxy::Event& event)
{
    if (_state != PROCESSING)
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
    if (_pending_events.empty())
    {
        return;
    }

    _current_event = std::move(_pending_events.front());
    _pending_events.pop();

    // Start async write - completion will trigger proceed()
    _responder.Write(_current_event, this);
}

//==============================================================================
// UpdateLedCallData Implementation
//==============================================================================

UpdateLedCallData::UpdateLedCallData(
    pin_proxy::PinProxyService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    GrpcUserFrontend* frontend)
    : _state(CREATE),
      _responder(&_ctx),
      _service(service),
      _cq(cq),
      _frontend(frontend)
{
    proceed();
}

void UpdateLedCallData::proceed()
{
    if (_state == CREATE)
    {
        _state = PROCESSING;
        _service->RequestUpdateLed(&_ctx, &_request, &_responder, _cq, _cq, this);
    }
    else if (_state == PROCESSING)
    {
        new UpdateLedCallData(_service, _cq, _frontend);

        int led_id = _request.led_id();
        bool active = _request.active();

        SENSEI_LOG_INFO("UpdateLed: led_id={}, active={}", led_id, active);

        _frontend->set_digital_output(led_id, active);

        _state = DONE;
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
    pin_proxy::PinProxyService::AsyncService* service,
    grpc::ServerCompletionQueue* cq,
    GrpcUserFrontend* frontend)
    : _state(CREATE),
      _responder(&_ctx),
      _service(service),
      _cq(cq),
      _frontend(frontend)
{
    proceed();
}

void RefreshAllStatesCallData::proceed()
{
    if (_state == CREATE)
    {
        _state = PROCESSING;
        _service->RequestRefreshAllStates(&_ctx, &_request, &_responder, _cq, _cq, this);
    }
    else if (_state == PROCESSING)
    {
        new RefreshAllStatesCallData(_service, _cq, _frontend);

        // TODO: Implement state query - for now return UNIMPLEMENTED
        SENSEI_LOG_WARNING("RefreshAllStates not implemented yet");

        _state = DONE;
        _responder.Finish(_response,
                         grpc::Status(grpc::StatusCode::UNIMPLEMENTED, "RefreshAllStates not implemented"),
                         this);
    }
    else
    {
        assert(_state == DONE);
        delete this;
    }
}

void EventBroadcastManager::register_subscriber(SubscribeCallData* subscriber)
{
    if (_shutting_down)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(_subscribers_mutex);
    _subscribers.push_back(subscriber);
    SENSEI_LOG_INFO("Subscriber registered, total subscribers: {}", _subscribers.size());
}

void EventBroadcastManager::unregister_subscriber(SubscribeCallData* subscriber)
{
    std::lock_guard<std::mutex> lock(_subscribers_mutex);
    auto it = std::find(_subscribers.begin(), _subscribers.end(), subscriber);
    if (it != _subscribers.end())
    {
        _subscribers.erase(it);
        SENSEI_LOG_INFO("Subscriber unregistered, total subscribers: {}", _subscribers.size());
    }
}

void EventBroadcastManager::broadcast_event(const pin_proxy::Event& event)
{
    if (_shutting_down)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(_subscribers_mutex);

    // Broadcast to all active subscribers
    for (auto* subscriber : _subscribers)
    {
        subscriber->enqueue_event(event);
    }
}

void EventBroadcastManager::shutdown()
{
    _shutting_down = true;
    SENSEI_LOG_INFO("EventBroadcastManager shutting down");
}

} // namespace user_frontend
} // namespace sensei
