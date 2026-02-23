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
 * @brief gRPC-based user frontend
 * @copyright 2017-2026 Elk Audio AB, Stockholm
 */
#ifndef SENSEI_GRPC_USER_FRONTEND_H
#define SENSEI_GRPC_USER_FRONTEND_H

#include <memory>
#include <thread>
#include <mutex>
#include <grpcpp/grpcpp.h>

#include "message/command_defs.h"
#include "user_frontend.h"
#include "sensei-grpc-api/sensei_rpc.pb.h"
#include "sensei-grpc-api/sensei_rpc.grpc.pb.h"

namespace sensei {
namespace user_frontend {

class GrpcUserFrontend;
class EventBroadcastManager;
class SubscribeCallData;
class UpdateLedCallData;
class RefreshAllStatesCallData;
class GetControllerMapCallData;

/**
 * @brief Async gRPC service implementation for SenseiController
 * Uses gRPC async API with completion queues for scalable concurrent connections
 */
class AsyncSenseiControllerImpl
{
public:
    AsyncSenseiControllerImpl(GrpcUserFrontend* frontend);
    ~AsyncSenseiControllerImpl();

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
    void broadcast_event(const sensei_rpc::Event& event);

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
    std::unique_ptr<sensei_rpc::SenseiController::AsyncService> _async_service;
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
    GrpcUserFrontend(MessageHandler* handler,
                     const int max_n_sensors,
                     ThreadingMode threading_mode = ThreadingMode::ASYNCHRONOUS);

    ~GrpcUserFrontend();

    /**
     * @brief Handle configuration commands
     */
    CommandErrorCode apply_command(const Command* cmd) override;

    /**
     * @brief Forward event from GrpcBackend to service for broadcasting
     * @param event The sensor event to broadcast to subscribers
     */
    void broadcast_event(const sensei_rpc::Event& event);

    /**
     * @brief Get the number of currently-connected subscribers, mostly for testing.
     */
    int num_subscribers() const;

    /**
     * @brief Update the sensor map with the name and type for this controller ID.
     * @param controller_id The ID of the controller to update
     * @param name The name of the controller
     * @param sensor_type The type of the controller
     */
    void update_controller(int controller_id, const std::string &name, SensorType type);

    /**
     * @brief Populate the controller map response with current sensor configuration
     * @param response The GetControllerMapResponse to populate
     */
    void populate_controller_map(sensei_rpc::GetControllerMapResponse* response);

    /**
     * @brief Trigger the MCU to re-send the values of all known controllers.
     */
    void refresh_controller_values();

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

    std::unique_ptr<AsyncSenseiControllerImpl> _service_impl;

    bool _server_running;
    std::mutex _server_mutex;

    struct Controller
    {
        std::string name;
        SensorType type{SensorType::UNDEFINED};
    };
    std::mutex _controller_map_mutex;
    std::vector<Controller> _controller_map;
};

} // namespace user_frontend
} // namespace sensei

#endif //SENSEI_GRPC_USER_FRONTEND_H
