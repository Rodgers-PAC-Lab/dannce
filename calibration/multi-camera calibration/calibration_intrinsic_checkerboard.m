%% Find the camera intrinsic parameters
% Input Parameters
clear
basedir = "/home/mouse/mnt/cuttlefish/lucas/3D_vids/Session_4/Video_12";
cd(basedir)
numcams = 5;
squareSize = 20.0; % Size of Checkerboard squares in mm
camera_ids = ["e3v833f" "e3v83e4" "e3v82eb" "e3v83d2" "e3v83d9"];
% camera_ids = ["e3v83d2"];
ext = ".mp4";
maxNumImages = 500;
videoName = "-20230724T134238-134450_smaller";
%% Automated Checkerboard Frame Detection
% Pre-allocate
params_individual = cell(1,numcams);
estimationErrors = cell(1,numcams);
imagePoints = cell(1,numcams);
boardSize = cell(1,numcams);
imagesUsed = cell(1,numcams);
imageNums = cell(1,numcams);

clear video_temp
for kk = 1:numcams
    
    tic
    video_temp = VideoReader(basedir+filesep+camera_ids(kk)+videoName+ext);
    maxFrames = floor(video_temp.FrameRate*video_temp.Duration);
    
    video_base = cell(maxFrames,1);
    cnt = 1;
    while hasFrame(video_temp)
        video_base{cnt} = readFrame(video_temp,'native');
        cnt = cnt + 1;
    end
    num2use = length(video_base);
    clear video_temp
        
    imUse1 = round(linspace(1,length(video_base),num2use));
    fprintf('finding checkerboard points for view %i \n', kk)
    [imagePoints{kk}, boardSize{kk}, imagesUsed{kk}] = ...
        detectCheckerboardPointsPar(cat(4,video_base{imUse1}), 'MinCornerMetric', 0.15);

    worldPoints = generateCheckerboardPoints(boardSize{kk},squareSize);
    imagesUsedTemp = find(imagesUsed{kk});
    numImagesToUse = min([maxNumImages numel(imagesUsedTemp)]);
    [~,imageNums{kk}] = datasample(imagesUsedTemp,numImagesToUse,'Replace',false);
    disp(['Images used for view ' num2str(kk) ': ' num2str(numel(imageNums{kk}))]);
    I = video_base{1};
    imageSize = [size(I,1),size(I,2)];
    [params_individual{kk},pairsUsed,estimationErrors{kk}] = estimateCameraParametersPar(imagePoints{kk}(:,:,imageNums{kk}),worldPoints, ...
        'ImageSize',imageSize,'EstimateSkew',true,'EstimateTangentialDistortion',true,...
        'NumRadialDistortionCoefficients',3);
    toc
end

% cd('../')
% int_file = load('cam_intrinsics5.mat');
% int_file.params_individual{4} = params_individual{1};
% int_file.imagePoints{4} = imagePoints{1};
% int_file.imagesUsed{4} = imagesUsed{1};
% int_file.imageNums{4} = imageNums{1};
% params_individual = int_file.params_individual;
% imagePoints = int_file.imagePoints;
% imagesUsed = int_file.imagesUsed;
% inmageNums = int_file.imageNums;
% Save the camera parameters
cd('../') %definitely refactor this
% save('cam_intrinsics6.mat','int_file.params_individual','int_file.imagePoints','int_file.boardSize','int_file.imagesUsed','int_file.imageNums');
save('cam_intrinsics7.mat','params_individual','imagePoints','boardSize','imagesUsed','imageNums');

%% Visualize Preprojections
cd(basedir)
cd('../')
load('cam_intrinsics7.mat')
numcams = 5;
for kk = 1:numcams
    video_temp = VideoReader(basedir+filesep+camera_ids(kk)+videoName+ext);    
    maxframes = floor(video_temp.FrameRate*video_temp.Duration);
    video_base = cell(maxframes,1);
    cnt = 1;
    while hasFrame(video_temp)
        video_base{cnt} = readFrame(video_temp,'native');
        cnt = cnt + 1;
    end
    
    clear M
    figure;
%     imagesUsed_ = find(imagesUsed{kk});
    imagesUsed_ = imageNums{kk};
    imagesUsedFull_ = find(imagesUsed{kk});
    imagesUsedFull_ = imagesUsedFull_(imagesUsed_);
    
    %show first 20 of each
%     for im2use = 1:numel(imagesUsed_)
    for im2use = 1:20
        imUsed = imagesUsed_(im2use);
        imDisp = imagesUsedFull_(im2use);
        pts = imagePoints{kk}(:,:,imUsed);
        repro = params_individual{kk}.ReprojectedPoints(:,:,im2use);
        imagesc(video_base{imDisp});colormap(gray)
        hold on;
        plot(pts(:,1),pts(:,2),'or');
        plot(repro(:,1),repro(:,2),'xg');
        drawnow;
        M(im2use) = getframe(gcf);
    end
    
    % write reproject video
    %vidfile = [basedir 'reproject_view' num2str(kk) '.mp4'];
    %vk = VideoWriter(vidfile);
    %vk.Quality = 100;
    %open(vk)
    %writeVideo(vk,M);
    %close(vk);
    
end
%% View Undistorted Images
load([basedir 'cam_intrinsics.mat'])
for kk=1:numcams
    imFiles1 = VideoReader(basedir+filesep+camera_ids(kk)+videoName+ext,'CurrentTime',0.5); 
    figure(kk);
    im = readFrame(imFiles1,'native');
    subplot(121);imagesc(im);
    subplot(122);imagesc(undistortImage(im,params_individual{kk}));
end























