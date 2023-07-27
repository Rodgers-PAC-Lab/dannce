%% Use predetermined crop ROI parameters to crop a single image of the
% calibration lframe We will use these
% cropped images to determine view extrinsic parameters.
clear
% Load in the crop params
basedir = "/home/mouse/mnt/cuttlefish/lucas/3D_vids/Session_4/Video_8";
intr_dir = "/home/mouse/mnt/cuttlefish/lucas/3D_vids/Session_4";
numCams = 5; 
numPoints = 5; %do we want numPoints = numCams-1? Seems like what they did in the paper. But, 2 points for 3 cameras seems small...
ext = '.tiff';
camera_ids = ["e3v833f" "e3v83e4" "e3v82eb" "e3v83d2" "e3v83d9"];
video_name = "-20230721T103338-103340_smaller";
cd(basedir)

% Input the physical (x,y,z) dimensions for each of the post tops on the 
% L-frame device.
% LFrame_coordinates = [ -8.5 5 1; 8.5 -5 1; 8.5 5 1; 0 0 1];%thoroughly perplexed by how these points translate into an L-frame... run commented LW code below to visualize
% LFrame_coordinates = [0 0 1; -8.5 5 1; 8.5 5 1; 8.5 -5 1; -8.5 -5 1];
% LFrame_coordinates = [-5 -5 2.5; 5 -5 4.5; -5 5 6.5; -1.5 0 8.5; 5 5 10.5];
LFrame_coordinates = [-5 -5 2.5; 5 -5 4.5; -5 5 6.5; -2.0 0 8.5; 5 5 10.5]; %I think the spire at (-1.5,0,8.5) is off by 0.5 cm
%LW added 6/8/23------------------------------------------------
% x = [-5 5 -5 0 5];
% y = [-5 -5 5 0 5];
% z = [2.5 4.5 6.5 8.5 10.5];
% figure()
% scatter3(x,y,z)
%---------------------------------------------------------------
LFrame_coordinates = 10*(LFrame_coordinates); % cm to mm

% convert first frame of each video to tiff
% fileloc = [basedir filesep 'extrinsic'];
% cd(fileloc)
for kk = 1:numCams
    filename = camera_ids(kk)+video_name;
    video_temp = VideoReader(filename+'.mp4','CurrentTime', 0);
    frame_temp = readFrame(video_temp);
%     imshow(frame_temp)
    imwrite(frame_temp,filename+".tiff")
end

lframe = cell(numCams,1);
% lframe = cell(numCams-1,1);
for i = 1:numCams
%     if i==3
%         continue
%     elseif i>3
%         filename = camera_ids(i)+video_name;
%         lframe{i-1} = imread(filename+ext);
% 
%     else
    filename = camera_ids(i)+video_name;
    lframe{i} = imread(filename+ext);
%     end
end
load(intr_dir + filesep + "cam_intrinsics7.mat");

%% Click points for each post top in order for each view
figure;
for kk = 1:numel(lframe)
    camparams = params_individual{kk};
    LFrame_image{kk} = undistortImage(lframe{kk},camparams);
    imagesc(LFrame_image{kk});colormap(gray)
    [xi,yi] = getpts;
    point_coordinates{kk} = [xi yi];
end

% Remove any extra points at the end that were accidentally marked
% for kk = 1:numel(lframe)
%     point_coordinates{kk} = point_coordinates{kk}(1:numPoints,:);
% end

% Plot point coordinates
figure;
for kk = 1:numel(lframe)
    hold off; imagesc(LFrame_image{kk});colormap(gray)
    hold on; plot(point_coordinates{kk}(:,1),point_coordinates{kk}(:,2),'or')
    waitforbuttonpress;
end

% If we mis-labeled a point, remove it here
% rmf = 5;
% for kk = 1:numel(lframe)
%     point_coordinates{kk}(4,:) = [];
% end

% Also remove from LFrame_coordinates
%LFrame_coordinates(5:end,:) = [];

%% Use selected points to calculate camera extrinsics
for kk = 1:numel(lframe)
    %kk
    % Do grid search over parms
    curr_err = 1e10;
    c_save = 0;
    mr_save = 0;
    mn_save = 10;
    for c = 99:-3:90
        %c
        for mr = [0.5 1 1.5 2 2.5 3 3.5 4 4.5 5:5:50]
            for mn = 1e10 
                try
                    [worldO,worldL] = estimateWorldCameraPose(double(point_coordinates{kk}),...
                        double(LFrame_coordinates),params_individual{kk},'Confidence',c,'MaxReprojectionError',mr,'MaxNumTrials',mn);
                    for i = 1:200
                        [worldO,worldL] = estimateWorldCameraPose(double(point_coordinates{kk}),...
                            double(LFrame_coordinates),params_individual{kk},'Confidence',c,'MaxReprojectionError',mr,'MaxNumTrials',mn);
                        [rotationM,translationV] = cameraPoseToExtrinsics(worldO,worldL);
                        imagePoints = worldToImage(params_individual{kk},rotationM,translationV,double(LFrame_coordinates));
                        err = mean(sqrt(sum((imagePoints-point_coordinates{kk}).^2,2)))
                        if err < curr_err
                            ["Found better repro error: " num2str(err)]
                            curr_err = err;
                            c_save = c;
                            mr_save = mr;
                            mn_save = mn;
                        end
                        
                    end
                catch ex
                    disp(ex)
                end

            end

        end
    end
    
    curr_err = 1e10;
    for i=1:200
        try
            [worldO,worldL] = estimateWorldCameraPose(double(point_coordinates{kk}),...
                double(LFrame_coordinates),params_individual{kk},'Confidence',c_save,'MaxReprojectionError',mr_save,'MaxNumTrials',mn_save);
            [rotationM,translationV] = cameraPoseToExtrinsics(worldO,worldL);
            imagePoints = worldToImage(params_individual{kk},rotationM,translationV,double(LFrame_coordinates));
            err = mean(sqrt(sum((imagePoints-point_coordinates{kk}).^2,2)));
            if err < curr_err
                rotationMatrix{kk} = rotationM;
                translationVector{kk} = translationV;
                worldOrientation{kk} = worldO;
                worldLocation{kk} = worldL;
                curr_err = err;
            end
        catch ex
            disp(ex)
        end
    end
    
    figure(222)
    plotCamera('Location',worldLocation{kk},'Orientation',worldOrientation{kk},'Size',50,'Label',num2str(kk));
    axis equal
    hold on
end
print('-dpng','cameraArrangement.png');

% Save full camera parameters: intrinsics, extrinsics
cd('../')
save("camera_params6",'params_individual','worldOrientation','worldLocation','rotationMatrix','translationVector');

%% Find closed form solution for rotation and translation.
% for kk = 1:numel(lframe)
% [rotationMatrix{kk}, translationVector{kk}] = extrinsics(double(point_coordinates{kk}),double(LFrame_coordinates(:,1:2)),params_individual{kk});
% end
%% Examine reprojections
%y_reflect = [-1 0 0; 0 -1 0; 0 0 1];
for kk = 1:numel(lframe)
    figure(233+kk)
    imagesc( LFrame_image{kk});colormap(gray);
    hold on
    
    tmp_v = translationVector{kk};
%     tmp_v(3) = -1*tmp_v(3);
%     tmp_v = -1*tmp_v;
    %     tmp_v
    tmp_r = rotationMatrix{kk};
%     tmp_r(:,1) = -1*tmp_r(:,1);
    imagePoints = worldToImage(params_individual{kk},tmp_r,tmp_v,double(LFrame_coordinates));
    
    colorarray = jet(size(imagePoints,1));%{'r','g','b','y','m','w','k'};
    for llll = 1:size(imagePoints,1)
        plot(imagePoints(llll,1),imagePoints(llll,2),'o','MarkerSize',4,...
            'MarkerEdgeColor',colorarray(llll,:),'MarkerFaceColor',colorarray(llll,:))
    end
    for llll = 1:size(imagePoints,1)
        plot(point_coordinates{kk}(llll,1),point_coordinates{kk}(llll,2),'x','MarkerSize',4,...
            'MarkerEdgeColor',colorarray(llll,:),'MarkerFaceColor',colorarray(llll,:))
    end
    hold off
    print('-dpng',basedir+"camerareproject"+camera_ids(kk)+".png");
    mean(sqrt(sum((imagePoints-point_coordinates{kk}).^2,2)))
end

%%camera to world image
for kk = 1:numCams
     figure(222)
     plotCamera('Location',worldLocation{kk},'Orientation',worldOrientation{kk},'Size',5,'Label',num2str(kk));
     hold on;
end
