import grpc
import sensei_rpc_pb2
import sensei_rpc_pb2_grpc

# these should be set to the appropriate controllers from the sensei config
DISABLE_BYPASS_CONTROLLER_ID = 3
ENABLE_MUTE_CONTROLLER_ID = 4

channel = grpc.insecure_channel('localhost:50051')
stub = sensei_rpc_pb2_grpc.SenseiControllerStub(channel)
stub.UpdateLed(sensei_rpc_pb2.UpdateLedRequest(controller_id=DISABLE_BYPASS_CONTROLLER_ID, active=True))
stub.UpdateLed(sensei_rpc_pb2.UpdateLedRequest(controller_id=ENABLE_MUTE_CONTROLLER_ID, active=False))
channel.close()