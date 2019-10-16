function  submap_ptidx = getSubmapAroundThePose(pose_se3, tree, topk)
    pose_xyz = [pose_se3(1,4), pose_se3(2,4), pose_se3(3,4)];
    submap_ptidx = knnsearch(tree, pose_xyz, 'K', topk);
end

