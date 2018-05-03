% Author: Jared Strader
% File: main_kodak.m
%
% Description: Computes the calibration parameters for the Kodak SP360 
% camera, then projects the example images on the unit sphere and 
% reprojects the unit sphere to a perspective image with the specified
% orientation. The reprojectied image is not interpolated. 
%
% x-axis -> right in the image aligned with u coordinates
% y-axis -> down in image aligned with v coordinates
% z-axis -> out of image (negative behind camera
% 
% Note: 
% -> Add all subfolders to path for code to run properly.
% -> The synthetic image was generated using blender with the calibration
% parameters computed in this script.
%
% https://www.mathworks.com/help/vision/ug/fisheye-calibration-basics.html
% https://sites.google.com/site/scarabotix/ocamcalib-toolbox
%
clear all; close all; clc;

%% Calibrate Fisheye (Kodak SP360)
% for i = 1:24
%   imageFileName = sprintf('Image%d.jpg', i);
%   imageFileNames{i} = fullfile(imageFileName);
% end
% 
% [imagePoints,boardSize] = detectCheckerboardPoints(imageFileNames);
% 
% squareSize = 22.225; % millimeters
% worldPoints = generateCheckerboardPoints(boardSize,squareSize);
% 
% I = imread('Image1.jpg');
% imageSize = [size(I,1) size(I,2)];
% params = estimateFisheyeParameters(imagePoints,worldPoints,imageSize);
% save();

%% Compute Mapping from Image to Unit Sphere
% Parameters from Matlab Toolbox (code is above)
load('kodak_matlab_results.mat');
nrows = params.Intrinsics.ImageSize(1);
ncols = params.Intrinsics.ImageSize(2);
a0 = params.Intrinsics.MappingCoefficients(1);
a2 = params.Intrinsics.MappingCoefficients(2);
a3 = params.Intrinsics.MappingCoefficients(3);
a4 = params.Intrinsics.MappingCoefficients(4);
xc = params.Intrinsics.DistortionCenter(1);
yc = params.Intrinsics.DistortionCenter(2);
c = params.Intrinsics.StretchMatrix(1,1);
d = params.Intrinsics.StretchMatrix(1,2);
e = params.Intrinsics.StretchMatrix(2,1);

% Create a grid containing the coordinates of every pixel in the image
[nx, ny] = meshgrid(1:nrows, 1:ncols);
n = [ny(:)';nx(:)'];

% Map all pixels to the unit sphere
num_points = size(n,2);
T = [xc;yc]*ones(1,num_points);
A = [c,d;e,1];
m = inv(A)*(n-T);
p = [a4, a3, a2, 0, a0];
M = [m(1,:) ; m(2,:) ; polyval(p,sqrt(m(1,:).^2+m(2,:).^2)) ];
M = normc(M)*1000;

% Plot normalized points
x=M(1,1:1:end);
y=M(2,1:1:end);
z=M(3,1:1:end);
% scatter3(x,y,z);
% axis equal

%% Project Image on Sphere
% Load and display image
figure;
I = im2double(imread('example1.JPG'));
imshow(I);

% Permute and reshape image to match the ordering of the generated mesh
J = permute(I,[2,1,3]); 
J = reshape(J,[],3);

% Display unit sphere with colored points
figure;
pcshow([x(:),y(:),z(:)],J);
xlabel('x');
ylabel('y');
zlabel('z');

%% Reproject to Perspective Image (using Logitech c920 parameters)
%load intrinsic parameters
load('logitech_c920_calibration_640x480.mat');
K = cameraParams.IntrinsicMatrix'; %transpose due to matlab convention

%choose orientation for perspective camera
thetax = -75*pi/180;
thetay = 0*pi/180;
thetaz = 45*pi/180;
Rx = [1,0,0;0,cos(thetax),-sin(thetax);0,sin(thetax),cos(thetax)];
Ry = [cos(thetay),0,sin(thetay);0,1,0;-sin(thetay),0,cos(thetay)];
Rz = [cos(thetaz),-sin(thetaz),0;sin(thetaz),cos(thetaz),0;0,0,1];
tf = Ry*Rx*Rz*M;
P = K*tf;

%reproject points in image
img = zeros(480,640,3);
for i=1:length(M) %this loop should be vectorized
    if(tf(3,i)>0) %ignore points behind camera
        pixel = P(:,i);
        w = pixel(3);
        u = round(pixel(1)/w);
        v = round(pixel(2)/w);
        if(u>0 && v>0 && u<=640 && v<=480)
            img(v,u,1) = J(i,1);
            img(v,u,2) = J(i,2);
            img(v,u,3) = J(i,3);
        end
    end
end
figure;
imshow(img);