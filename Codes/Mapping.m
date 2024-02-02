image=imread('MAP.jpg');
image=rgb2gray(image);
image=medfilt2(image);
image=medfilt2(image);

imshow(image)
imageCropped = image(200:end-200,300:1000);
imshow(imageCropped)
imageBW = imageCropped <100;
imshow(imageBW)
map = binaryOccupancyMap(imageBW,0.25);
show(map)
robotRadius = 1;
mapInflated = copy(map);
inflate(mapInflated,robotRadius);
show(mapInflated)
%planning
prm = mobileRobotPRM;
prm.Map = mapInflated;
prm.NumNodes = 1;
prm.ConnectionDistance = 100;
startLocation = [1204 6282];
endLocation = [1569 6987];
path = findpath(prm, startLocation, endLocation)
show(prm)
while isempty(path)
    prm.NumNodes = prm.NumNodes + 1000
    update(prm);
    path = findpath(prm, startLocation, endLocation)
end
show(prm)