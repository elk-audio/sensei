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
 * @brief Async gRPC call handlers for PinProxyService
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_ASYNC_CALL_HANDLERS_H
#define SENSEI_ASYNC_CALL_HANDLERS_H

#include <mutex>
#include <queue>
#include <grpcpp/grpcpp.h>

#include "pin-proxy/pin_events.pb.h"
#include "pin-proxy/pin_events.grpc.pb.h"

namespace sensei {
namespace user_frontend {

class GrpcUserFrontend;
class EventBroadcastManager;

/**
 * @brief Base class for async RPC call handlers
 * Provides common proceed() interface for completion queue processing
 */
class CallDataBase
{
public:
    enum State { CREATE, PROCESSING, DONE };

    virtual ~CallDataBase() = default;
    virtual void proceed() = 0;
    virtual void stop() {};
};

/**
 * @brief Async handler for SubscribeToEvents server-streaming RPC
 */
class SubscribeCallData : public CallDataBase
{
public:
    SubscribeCallData(
        pin_proxy::PinProxyService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        EventBroadcastManager* broadcast_mgr,
        GrpcUserFrontend* frontend);

    void proceed() override;

    void stop() override;

    /**
     * @brief Called by EventBroadcastManager to queue event for this subscriber
     * @param event The event to queue
     */
    void enqueue_event(const pin_proxy::Event& event);

private:
    /**
     * @brief Start writing next event from queue to client
     * Must be called with _write_mutex held
     */
    void _start_write();

    State _state;
    grpc::ServerContext _ctx;
    pin_proxy::SubscribeRequest _request;
    grpc::ServerAsyncWriter<pin_proxy::Event> _responder;

    pin_proxy::PinProxyService::AsyncService* _service;
    grpc::ServerCompletionQueue* _cq;
    EventBroadcastManager* _broadcast_mgr;
    GrpcUserFrontend* _frontend;

    // Event queue and write serialization
    std::mutex _write_mutex;
    std::queue<pin_proxy::Event> _pending_events;
    pin_proxy::Event _current_event;
    bool _in_processing;
};

/**
 * @brief Async handler for UpdateLed unary RPC
 */
class UpdateLedCallData : public CallDataBase
{
public:
    UpdateLedCallData(
        pin_proxy::PinProxyService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        GrpcUserFrontend* frontend);

    void proceed() override;

private:
    State _state;
    grpc::ServerContext _ctx;
    pin_proxy::UpdateLedRequest _request;
    pin_proxy::GenericVoidValue _response;
    grpc::ServerAsyncResponseWriter<pin_proxy::GenericVoidValue> _responder;

    pin_proxy::PinProxyService::AsyncService* _service;
    grpc::ServerCompletionQueue* _cq;
    GrpcUserFrontend* _frontend;
};

/**
 * @brief Async handler for RefreshAllStates unary RPC
 */
class RefreshAllStatesCallData : public CallDataBase
{
public:
    RefreshAllStatesCallData(
        pin_proxy::PinProxyService::AsyncService* service,
        grpc::ServerCompletionQueue* cq,
        GrpcUserFrontend* frontend);

    void proceed() override;

private:
    State _state;
    grpc::ServerContext _ctx;
    pin_proxy::RefreshAllStatesRequest _request;
    pin_proxy::RefreshAllStatesResponse _response;
    grpc::ServerAsyncResponseWriter<pin_proxy::RefreshAllStatesResponse> _responder;

    pin_proxy::PinProxyService::AsyncService* _service;
    grpc::ServerCompletionQueue* _cq;
    GrpcUserFrontend* _frontend;
};

/**
 * @brief Thread-safe manager for broadcasting events to multiple subscribers
 *
 * Maintains list of active subscribers and provides thread-safe broadcast operation
 */
class EventBroadcastManager
{
public:
    EventBroadcastManager() : _shutting_down(false) {}
    ~EventBroadcastManager() = default;

    /**
     * @brief Register a new subscriber
     * @param subscriber Non-owning pointer to SubscribeCallData
     */
    void register_subscriber(SubscribeCallData* subscriber);

    /**
     * @brief Unregister a subscriber (called when client disconnects)
     * @param subscriber Pointer to SubscribeCallData to remove
     */
    void unregister_subscriber(SubscribeCallData* subscriber);

    /**
     * @brief Broadcast event to all active subscribers
     * @param event The event to broadcast
     */
    void broadcast_event(const pin_proxy::Event& event);

    /**
     * @brief Signal shutdown - prevents new broadcasts
     */
    void shutdown();

private:
    std::mutex _subscribers_mutex;
    std::vector<SubscribeCallData*> _subscribers;  // Non-owning pointers
    std::atomic<bool> _shutting_down;
};

} // namespace user_frontend
} // namespace sensei

#endif // SENSEI_ASYNC_CALL_HANDLERS_H
