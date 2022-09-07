yaw_in = linspace(0,2*pi,10);

yaw_out = zeros(10,3);
quats = zeros(10,4);
quat = quaternion.empty;
for i = 1:10
    eul = [0 yaw_in(i) 0];
    quats(i,:) = eul2quat(eul);
    quat(i) = quaternion(eul,'rotvec');
    yaw_out(i,:) = euler(quat(i),'YXZ','frame');
end