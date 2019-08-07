%  USAGE: data = readlog(filename); 
%    read MuJoCo HAPTIX data log file (.mjl)

function data = readlog(filename)

    % open file
    fid = fopen(filename,'r');
    if ~fid,
        error('Could not open file');
    end

    % read header
    tmp = fread(fid, 6, 'int32');
    if tmp(6)>1,
        name = char(fread(fid, tmp(6))');
    else
        name = '';
    end
    header = struct('nq', tmp(1), 'nv', tmp(2), 'nu', tmp(3), 'nmocap', tmp(4), 'nsensordata', tmp(5), 'name', name);

    % read data, close
    num = fread(fid, 'single');
    fclose(fid);

    % reshape numeric data
    recsz = 1 + header.nq + header.nv + header.nu + 7*header.nmocap + header.nsensordata;
    if mod(length(num), recsz)
        error('Data size is not divisible by record size');
    end
    num = reshape(num, [recsz length(num)/recsz]);

    % construct data -- incorporate header
    data = struct('nq', tmp(1), 'nv', tmp(2), 'nu', tmp(3), 'nmocap', tmp(4), 'nsensordata', tmp(5), 'name', name,...
        'time', num(1,:), ...
        'qpos', num(2:(header.nq+1),:), ...
        'qvel', num((header.nq+2):(header.nq+header.nv+1),:), ...
        'ctrl', num((header.nq+header.nv+2):(header.nq+header.nv+header.nu+1),:), ...
        'mocap_pos', num((header.nq+header.nv+header.nu+2):(header.nq+header.nv+header.nu+3*header.nmocap+1),:), ...
        'mocap_quat', num((header.nq+header.nv+header.nu+3*header.nmocap+2):(header.nq+header.nv+header.nu+7*header.nmocap+1),:), ...
        'sensordata', num((header.nq+header.nv+header.nu+7*header.nmocap+2):end,:));

end
