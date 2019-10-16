%% Info
% author: Giseop Kim (paulgkim@kaist.ac.kr)
% date: 2019. 10. 16

%% NOTE
% 1. memory: this program may require the memory 5-10GB
% 2. time cost (example below: urban09 has total 80,000,000 points):
% - Reading LAS file into array: 1-2 min 
% - Making KD-tree: 3-4 min 
% - Making a single scan: ~5 sec
%   - parsing a single submap: ~3 sec for 5m points
%   - projecting the submap into a (virtual) horizontal scan: ~2 sec 

%%
clear; clc;
addpath(genpath('.'));
format long

%% Configuration  
config_projection; % also imoported in the function 'submap2horizontalscan'
SCAN_HZ = 2; % user parameter 

save_pcd = 0;
save_npy = 1;

%% Download link: http://irap.kaist.ac.kr/dataset/download_1.html

% % - Pose 
gtposes_path = '../data/urban09/global_pose.csv';
gtposes = csvread(gtposes_path);
num_poses = length(gtposes); % e.g., 77319953 points (2.6GB) in urban09

% - Point cloud (LAS format)
lasfile_path = '../data/urban09/sick_pointcloud.las';
LasStruct = lasread(lasfile_path); % may takes 1-2 minutes for 2.6GB LAS file (urban09)
whole_map_xyz = ([LasStruct.X, LasStruct.Y, LasStruct.Z]);
whole_map_xyz = pcdownsample(pointCloud(whole_map_xyz),'gridAverage', 0.01); % remove almost repeated points
whole_map_xyz = whole_map_xyz.Location;
clear LasStruct; % for memory save 
disp("A whole point cloud is loaded.");

% tree for fast search 
tree = createns(whole_map_xyz); % may takes 3-4 minutes for 2.6GB LAS file (urban09)
disp("The point cloud's KD tree is made.");

% viz
whole_map_xyz_down = pcdownsample(pointCloud(whole_map_xyz), 'random', 0.01); % remaining 1%
figure(1); clf;
pcshow(whole_map_xyz_down);

%% Main 
mkdir scans_npy
mkdir scans_pcd

clear video_frames;
video_frame_idx = 1;

time_accumulator = 0; % take a scan according to the HZ

START_IDX = 100;
prev_pose_time_sec = gtposes(START_IDX, 1) * 1e-9;
for pose_idx = START_IDX+1:num_poses
    tic

    % watching time for taking keyframe (e.g., 10Hz)
    curr_pose_time = gtposes(pose_idx, 1); 
    curr_pose_time_sec = curr_pose_time * 1e-9;
    time_accumulator = time_accumulator + (curr_pose_time_sec - prev_pose_time_sec);
    prev_pose_time_sec = curr_pose_time_sec;
    if(time_accumulator < (1/SCAN_HZ)) % take a scan according to the HZ
        continue;
    else % ACTAUL MAIN 
        pose_se3 = gtposes(pose_idx, 2:end); 
        pose_se3 = [transpose(reshape(pose_se3, 4, 3)); 0,0,0,1];

        % taking submap near the pose 
        topk = 7000000; % empirically enough.
        submap_ptidx = getSubmapAroundThePose(pose_se3, tree, topk);
        submap = whole_map_xyz(submap_ptidx, :);
        submap_homg = [submap'; ones(1, length(submap))];
        submap_localcoord = pose_se3 \ submap_homg; % == inv(pose_se3) * submap_homg;
        submap_localcoord = submap_localcoord(1:3, :)';
        if(0) % default off cause very time consuming in matlab for more than 1m points.
            figure(3); clf; 
            pcshow(pcdownsample(pointCloud(submap_localcoord), 'random', 0.1)); 
        end
        
        % making a horizontal scan via submap projection
        lidar_height = 1.9; % because ComplexUrbanDataset's GT z is on the ground.
        submap_localcoord_height_adjusted = submap_localcoord;
        submap_localcoord_height_adjusted(:, 3) = submap_localcoord_height_adjusted(:, 3) - lidar_height;
        scan = submap2horizontalscan(submap_localcoord_height_adjusted);
        
        % save scan as a certain format you want  
        time_str = num2str(uint64(curr_pose_time)); 
        if(save_pcd) 
            save_name = strcat('scans_pcd/scan_', time_str, '.pcd');
            pcwrite(pointCloud(scan), save_name, 'Encoding','binary');
        end
        if(save_npy) 
            save_name = strcat('scans_npy/scan_', time_str, '.npy');
            writeNPY(scan, save_name);
        end
                
        % viz for debugging 
        figure(4); clf; 
        pcshow(scan);
        xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
        VIZ_RANGE = 90;
        xlim([-VIZ_RANGE, VIZ_RANGE]); ylim([-VIZ_RANGE, VIZ_RANGE]);
        zlim([-5, 30]); 
        colormap jet; caxis([0, 10]);
        
        % save figure 
        video_frames(video_frame_idx) = getframe(gcf);
        video_frame_idx = video_frame_idx + 1;
        
        % renewal the flag
        time_accumulator= 0;
        
    end

    toc
end

%% example video saver 
writerObj = VideoWriter('scans.avi');
writerObj.FrameRate = 10;
open(writerObj);
for i=1:length(video_frames)
    frame = video_frames(i) ;    
    writeVideo(writerObj, frame);
end
close(writerObj);
