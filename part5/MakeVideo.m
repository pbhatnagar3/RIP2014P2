% Make video from image files
imageNames = dir(fullfile('im','','*.png'));
imageNames = {imageNames.name}';
outputVideo = VideoWriter(fullfile('rawImages','tree_RRT.avi'));
outputVideo.FrameRate = 30;
open(outputVideo)
for ii = 1:length(imageNames)
   img = imread(fullfile('rawImages','',imageNames{ii}));
   writeVideo(outputVideo,img)
end
close(outputVideo)