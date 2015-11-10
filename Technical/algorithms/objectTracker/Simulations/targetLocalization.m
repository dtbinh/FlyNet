%% Clean Up
close all;
clear all;
clc;

%% FLiR image
hPxFlir = 80;       % Width in pixels
vPxFlir = 60;       % Height in pixels

hFovFlir = 51*pi/180;      % Horizontal FOV [deg]
vFovFlir = 37.83*pi/180;   % Vertical FOV [deg]

pos = [2; 3; 4.5];
los = ([0,1,0]/norm([0,1,0]))';

imgFlir = zeros(vPxFlir, hPxFlir, 3);
tic;
fprintf('Progress:\n');
for rowIdx=1:vPxFlir
    for colIdx=1:hPxFlir
        az = hFovFlir/2 - (colIdx - 1)*hFovFlir/hPxFlir;
        el = vFovFlir/2 - (rowIdx - 1)*vFovFlir/vPxFlir;
        [intersectPoint] = pixelFromBearing(az, el, pos, los);
        if intersectPoint.valid == 1
            imgFlir(rowIdx, colIdx, 1:3) = intersectPoint.color;
        else
            fprintf('INVALID az = %f\tel = %f\trowIdx = %f\tcolIdx = %f\n', az, el, rowIdx, colIdx);
            imgFlir(rowIdx, colIdx, 1:3) = [0, 0, 0];
        end
    end
    fprintf('%f%%\n', ((rowIdx - 1)*hPxFlir + colIdx)*100/hPxFlir/vPxFlir);
end
toc

imshow(imgFlir/255);
imwrite(imgFlir/255, 'flir.jpg');

drawnow();

%% Depth image

vPxDepth = 240;
hPxDepth = 320;

hFovDepth = hFovFlir;
vFovDepth = vFovFlir;

imgFlir = zeros(vPxDepth, hPxDepth);

tic;
fprintf('Progress:\n');
for rowIdx=1:vPxDepth
    for colIdx=1:hPxDepth
        az = hFovFlir/2 - (colIdx - 1)*hFovFlir/hPxDepth;
        el = vFovFlir/2 - (rowIdx - 1)*vFovFlir/vPxDepth;
        [intersectPoint] = pixelFromBearing(az, el, pos, los);
        if intersectPoint.valid == 1
            imgDepth(rowIdx, colIdx) = (1 + abs(randn()/30))*intersectPoint.dist/30;
        else
            imgDepth(rowIdx, colIdx) = [0];
        end
    end
    fprintf('%f%%\n', ((rowIdx - 1)*hPxDepth + colIdx)*100/hPxDepth/vPxDepth);
end
toc;

imshow(imgDepth);
imwrite(imgDepth, 'depth.jpg');