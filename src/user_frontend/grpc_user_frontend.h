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
 * @brief gRPC-based user frontend
 * @copyright 2017-2019 Modern Ancient Instruments Networked AB, dba Elk, Stockholm
 */
#ifndef SENSEI_GRPC_USER_FRONTEND_H
#define SENSEI_GRPC_USER_FRONTEND_H

#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <grpcpp/grpcpp.h>

#include "user_frontend.h"
#include "pin-proxy/pin_events.pb.h"
#include "pin-proxy/pin_events.grpc.pb.h"

namespace sensei {
namespace user_frontend {

class GrpcUserFrontend;
class EventBroadcastManager;
class SubscribeCallData;
class UpdateLedCallData;
class RefreshAllStatesCallData;

/**
 * @brief Async gRPC service implementation for PinProxyService
 * Uses gRPC async API with completion queues for scalable concurrent connections
 */
class AsyncPinProxyServiceImpl
{
public:
    AsyncPinProxyServiceImpl(GrpcUserFrontend* frontend);
    ~AsyncPinProxyServiceImpl();

    /**
     * @brief Start the async service and spawn worker threads
     * @param server_address Address to bind server (e.g., "0.0.0.0:50051")
     */
    void start(const std::string& server_address);

    /**
     * @brief Stop the async service and cleanup
     */
    void shutdown();

    /**
     * @brief Broadcast event to all active subscribers
     * Called by GrpcBackend via GrpcUserFrontend
     * @param event The event to broadcast
     */
    void broadcast_event(const pin_proxy::Event& event);

    /**
     * @brief Get the async service for handler registration
     */
    pin_proxy::PinProxyService::AsyncService* service() { return _async_service.get(); }

    /**
     * @brief Get the completion queue for handler registration
     */
    grpc::ServerCompletionQueue* cq() { return _cq.get(); }

    /**
     * @brief Get pointer to frontend for callbacks
     */
    GrpcUserFrontend* frontend() { return _frontend; }

    /**
     * @brief Get broadcast manager for subscriber registration
     */
    EventBroadcastManager* broadcast_manager() { return _broadcast_manager.get(); }

private:
    /**
     * @brief Worker thread function - processes completion queue events
     */
    void _handle_rpcs();

    GrpcUserFrontend* _frontend;
    std::unique_ptr<grpc::Server> _server;
    std::unique_ptr<grpc::ServerCompletionQueue> _cq;
    std::unique_ptr<pin_proxy::PinProxyService::AsyncService> _async_service;
    std::unique_ptr<EventBroadcastManager> _broadcast_manager;
    std::thread _cq_thread;
    bool _running;
};

/**
 * @brief gRPC-based user frontend
 * Manages gRPC server for both control input and sensor output streaming
 */
class GrpcUserFrontend : public UserFrontend
{
public:
    GrpcUserFrontend(SynchronizedQueue<std::unique_ptr<BaseMessage>>* queue,
                     const int max_n_input_pins,
                     const int max_n_digital_out_pins);

    ~GrpcUserFrontend();

    /**
     * @brief Handle configuration commands
     */
    CommandErrorCode apply_command(const Command* cmd) override;

    /**
     * @brief Forward event from GrpcBackend to service for broadcasting
     * @param event The sensor event to broadcast to subscribers
     */
    void broadcast_event(const pin_proxy::Event& event);

private:
    /**
     * @brief Start the gRPC async server
     */
    void _start_server();

    /**
     * @brief Stop the gRPC async server
     */
    void _stop_server();

    std::string _listen_address;
    int _listen_port;

    std::unique_ptr<AsyncPinProxyServiceImpl> _service_impl;

    bool _server_running;
    std::mutex _server_mutex;
};

} // namespace user_frontend
} // namespace sensei

#endif //SENSEI_GRPC_USER_FRONTEND_H
