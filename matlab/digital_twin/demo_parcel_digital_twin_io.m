function demo_parcel_digital_twin_io()
%DEMO_PARCEL_DIGITAL_TWIN_IO Minimal dry run for ROS2/YOLO twin I/O.

addpath(fileparts(mfilename('fullpath')));

parcel_manual_config("reset");
S = parcel_manual_core_step("reset", 0);

schema = parcel_digital_twin_io_adapter("schema");
disp("Digital twin I/O schema:");
disp(schema.version);
disp(schema.tofOrder);

rawFloor = parcel_digital_twin_io_adapter("ros_floor_template");
rawFloor.floorId = 1;
rawFloor.tofMm = [250 110 250 120 250 130 250 140];
rawFloor.tofOk = true(1,8);
rawFloor.encoderCount = [0 0 0 0];
rawFloor.mmPerEncoderCount = [0.10 0.10 0.10 0.10];
rawFloor.b4BarrierServoDeg = 0;
rawFloor.b4BarrierState = "DOWN";
floorState = parcel_digital_twin_io_adapter("normalize_ros_floor", rawFloor);
disp("Normalized floor state:");
disp(floorState.floorId);
disp(floorState.tofGapMm);

yolo = parcel_digital_twin_io_adapter("yolo_template");
yolo.parcelId = 101;
yolo.longMm = 270;
yolo.shortMm = 180;
yolo.heightMm = 150;
yolo.yawDeg = 3.5;
yolo.confidence = 0.91;
yolo.qrText = "DEMO-101";
yolo.destinationCode = "NORMAL";
yolo.isFirstDelivery = false;

S = parcel_digital_twin_io_adapter("apply_yolo_load", yolo, "normal");
for k = 1:8000
    S = parcel_manual_core_step("step", 0);
    if S.isIdle
        break;
    end
end

cmd = parcel_digital_twin_io_adapter("snapshot_to_command", S);
disp("Command frame ready:");
disp(cmd.readyToPublish);
disp(cmd.motorDir);
disp(cmd.encoderDeltaMm);
disp(S.message);
end
