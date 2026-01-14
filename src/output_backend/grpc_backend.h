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
 * @brief Output backend with gRPC - forwards events to user frontend
 * @copyright 2017-2026 Elk Audio AB, Stockholm
 */
#ifndef SENSEI_GRPC_BACKEND_H
#define SENSEI_GRPC_BACKEND_H

#include "output_backend.h"
#include "sensei-grpc-api/sensei_rpc.pb.h"

namespace sensei {
namespace user_frontend {
    class GrpcUserFrontend;
}
}

namespace sensei {
namespace output_backend {

/**
 * @brief gRPC backend for SENSEI - forwards sensor events to GrpcUserFrontend for streaming
 */
class GrpcBackend : public OutputBackend
{
public:
    GrpcBackend(const int max_n_input_pins = 64);
    ~GrpcBackend();

    /**
     * @brief Handle configuration commands
     */
    CommandErrorCode apply_command(const Command* cmd) override;

    /**
     * @brief Send sensor data by forwarding to GrpcUserFrontend
     * @param transformed_value The processed output value
     * @param raw_input_value The raw sensor value (not implemented yet)
     */
    void send(const OutputValue* transformed_value, const Value* raw_input_value) override;

    /**
     * @brief Set the user frontend to forward events to
     * @param frontend Pointer to the GrpcUserFrontend instance
     */
    void set_user_frontend(user_frontend::GrpcUserFrontend* frontend);

private:
    /**
     * @brief Convert SENSEI sensor types and values to proto Event messages
     * @param sensor_index The sensor index
     * @param sensor_type The SENSEI sensor type
     * @param value The float value to send
     * @param timestamp The timestamp in microseconds
     * @return A proto Event message
     */
    sensei_rpc::Event _create_proto_event(int sensor_index,
                                          SensorType sensor_type,
                                          float value,
                                          uint32_t timestamp);

    user_frontend::GrpcUserFrontend* _user_frontend;  // Non-owning pointer
};

} // namespace output_backend
} // namespace sensei

#endif //SENSEI_GRPC_BACKEND_H
