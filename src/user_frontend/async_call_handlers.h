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
 * @brief Async gRPC call handlers for SenseiController
 * @copyright 2017-2026 Elk Audio AB, Stockholm
 */
#ifndef SENSEI_ASYNC_CALL_HANDLERS_H
#define SENSEI_ASYNC_CALL_HANDLERS_H

#include <mutex>
#include <queue>
#include <grpcpp/grpcpp.h>

#include "sensei-grpc-api/sensei_rpc.pb.h"
#include "sensei-grpc-api/sensei_rpc.grpc.pb.h"

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
    CallDataBase(sensei_rpc::SenseiController::AsyncService* service,
                 grpc::ServerCompletionQueue*                cq)
        : _state(State::CREATE),
          _service(service),
          _cq(cq)
    {}

    virtual ~CallDataBase() = default;
    virtual void proceed() = 0;
    virtual void stop()
    {
        _state = State::DONE;
    }

protected:
    enum class State
    {
        CREATE,
        PROCESSING,
        DONE
    };

    State                                       _state;
    grpc::ServerContext                         _ctx;
    sensei_rpc::SenseiController::AsyncService* _service;
    grpc::ServerCompletionQueue*                _cq;
};

/**
 * @brief Async handler for SubscribeToEvents server-streaming RPC
 */
class SubscribeCallData : public CallDataBase
{
public:
    SubscribeCallData(
            sensei_rpc::SenseiController::AsyncService* service,
            grpc::ServerCompletionQueue*                cq,
            EventBroadcastManager*                      broadcast_mgr);

    void proceed() override;

    void stop() override;

    /**
     * @brief Called by EventBroadcastManager to queue event for this subscriber
     * @param event The event to queue
     */
    void enqueue_event(const sensei_rpc::Event& event);

private:
    /**
     * @brief Start writing next event from queue to client
     * Must be called with _write_mutex held
     */
    void _start_write();

    sensei_rpc::SubscribeRequest               _request;
    grpc::ServerAsyncWriter<sensei_rpc::Event> _responder;
    EventBroadcastManager*                     _broadcast_mgr;

    // Event queue and write serialization
    std::mutex                    _write_mutex;
    std::queue<sensei_rpc::Event> _pending_events;
    sensei_rpc::Event             _current_event;
    bool                          _in_processing;
    bool                          _is_writing;
};

/**
 * @brief Async handler for UpdateLed unary RPC
 */
class UpdateLedCallData : public CallDataBase
{
public:
    UpdateLedCallData(
            sensei_rpc::SenseiController::AsyncService* service,
            grpc::ServerCompletionQueue*                cq,
            GrpcUserFrontend*                           frontend);

    void proceed() override;

private:
    sensei_rpc::UpdateLedRequest                                  _request;
    sensei_rpc::GenericVoidValue                                  _response;
    grpc::ServerAsyncResponseWriter<sensei_rpc::GenericVoidValue> _responder;
    GrpcUserFrontend*                                             _frontend;
};

/**
 * @brief Async handler for RefreshAllStates unary RPC
 */
class RefreshAllStatesCallData : public CallDataBase
{
public:
    RefreshAllStatesCallData(
            sensei_rpc::SenseiController::AsyncService* service,
            grpc::ServerCompletionQueue*                cq,
            GrpcUserFrontend*                           frontend);

    void proceed() override;

private:
    sensei_rpc::GenericVoidValue                                  _request;
    sensei_rpc::GenericVoidValue                                  _response;
    grpc::ServerAsyncResponseWriter<sensei_rpc::GenericVoidValue> _responder;
    GrpcUserFrontend*                                             _frontend;
};

/**
 * @brief Async handler for GetControllerMap unary RPC
 */
class GetControllerMapCallData : public CallDataBase
{
public:
    GetControllerMapCallData(
            sensei_rpc::SenseiController::AsyncService* service,
            grpc::ServerCompletionQueue*                cq,
            GrpcUserFrontend*                           frontend);

    void proceed() override;

private:
    sensei_rpc::GenericVoidValue                                          _request;
    sensei_rpc::GetControllerMapResponse                                  _response;
    grpc::ServerAsyncResponseWriter<sensei_rpc::GetControllerMapResponse> _responder;
    GrpcUserFrontend*                                                     _frontend;
};

/**
 * @brief Thread-safe manager for broadcasting events to multiple subscribers
 *
 * Maintains list of active subscribers and provides thread-safe broadcast operation
 */
class EventBroadcastManager
{
public:
    EventBroadcastManager()
        : _shutting_down(false)
    {}
    ~EventBroadcastManager() = default;

    /**
     * @brief Register a new subscriber
     * @param subscriber Non-owning pointer to SubscribeCallData
     */
    void register_subscriber(SubscribeCallData* subscriber, std::vector<int>&& controller_ids);

    /**
     * @brief Unregister a subscriber (called when client disconnects)
     * @param subscriber Pointer to SubscribeCallData to remove
     */
    void unregister_subscriber(SubscribeCallData* subscriber);

    /**
     * @brief Broadcast event to all active subscribers
     * @param event The event to broadcast
     */
    void broadcast_event(const sensei_rpc::Event& event);

    /**
     * @brief Signal shutdown - prevents new broadcasts
     */
    void shutdown();

    /**
     * @brief Get the number of currently-connected subscribers
     */
    int num_subscribers();

private:
    struct SubscriberData
    {
        SubscribeCallData* subscriber; // non-owning pointer
        std::vector<int>   controller_ids;
    };

    std::mutex                  _subscribers_mutex;
    std::vector<SubscriberData> _subscribers;
    std::atomic<bool>           _shutting_down;
};

} // namespace user_frontend
} // namespace sensei

#endif // SENSEI_ASYNC_CALL_HANDLERS_H
