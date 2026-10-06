function sub_img_linear_move_F12_5
student_coeficient_value = 2.0452;
confidencelevel = 0.95;
n = 30;
pixel2Micron = 1.55;
resultfile = fopen("~/Documents/sensor fusion results/txtresults/Linear_Move_F12-5_results.txt","w");
img = cell(1, n);
dx = zeros(1, n-1);
dy = zeros(1, n-1);
r = zeros(1, n-1);

% Read images
for i = 1:n
    file = sprintf('~/Downloads/lucian sensorfusion/linear_move_f12.5/mlinp%d.bmp',i);
    img{i} = im2double(rgb2gray(imread(file)));
end

% Compute motion
for i = 2:n
    % Pixel-wise difference
    diffImage = abs(img{i} - img{i-1});
    bw = diffImage > 0.000001;      % threshold for motion
    bw = bwareaopen(bw, 50);      % remove small noise

    % Display frames and motion
    figure; 
    subplot(1,4,1);imshowpair(img{i-1}, img{i}, 'montage'); 
    title(sprintf('Frame %d vs Frame %d', i-1, i));
    
    subplot(1,4,2); imshow(bw, []); 
    title(sprintf('Detected Motion %d → %d', i-1, i));
subplot(1,4,3); imhist(img{i -1}); title(sprintf('hist of img: %d',i-1));
subplot(1,4,4); imhist(img{i}); title(sprintf('hist of img: %d',i));
    % Threshold to get object for centroid calculation

    bw1 = img{i-1} > graythresh(img{i-1});
    bw2 = img{i} > graythresh(img{i});
    s1 = regionprops(bw1, 'Centroid');
    s2 = regionprops(bw2, 'Centroid');

    if ~isempty(s1) && ~isempty(s2)
        c1 = s1(1).Centroid;
        c2 = s2(1).Centroid;
        fprintf(resultfile,"%d\tcentroid data \t|x1: %0.4f\t|y1: %0.4f\t|x2: %0.4f\t|y2: %0.4f\t|_|\t",i-1,c1(1),c1(2),c2(1),c2(2));
        dx(i-1) = c2(1) - c1(1);
        dy(i-1) = c2(2) - c1(2);
        r(i-1) = sqrt(dx(i-1)^2 + dy(i-1)^2) * pixel2Micron;
    else
        dx(i-1) = NaN;
        dy(i-1) = NaN;
        r(i-1) = NaN;
    end

    fprintf(resultfile,'Frame %d → %d: \t|dx = %.2f px,\t|dy = %.2f px,\t|r = %.2f μm\n',i-1, i, dx(i-1), dy(i-1), r(i-1));

end
meanR = mean(r, 'omitnan');
fprintf(resultfile,"------------------------------------------------\n");
fprintf(resultfile,'Mean motion radius: %.2f μm\n', meanR);
%standard deviationof the motion radius
stdR = std(r, 'omitnan');
fprintf(resultfile,'Standard deviation of motion radius: %.2f μm\n', stdR);
% calculate the absolut error
% Calculate the absolute error for dx and dy
absErrorDx = abs(dx - mean(dx, 'omitnan'));
absErrorDy = abs(dy - mean(dy, 'omitnan'));
absErrorR = abs(r - mean(r, 'omitnan'));
% probability of error
probabilityDx = 1/n*sum(absErrorDx);
% Calculate the probability of error for dy
probabilityDy = 1/n*sum(absErrorDy);
% Calculate the probability of error for the motion radius
mean_abs_errorR = 1/n*sum(absErrorR);
fprintf('mean of absolute eror: %f\n',mean_abs_errorR);
% experimental standasd mean deviation
s_md = stdR/sqrt(n);
fprintf('standard mean deviation: %f\n',s_md);
% random error

rand_Error = student_coeficient_value*s_md;
fprintf('random error: %f',rand_Error);
% Display results in table
motionData = table((1:n-1)', (dx*pixel2Micron)', (dy*pixel2Micron)', r',absErrorDx',absErrorDy',absErrorR', 'VariableNames', {'Frame', 'dx [µm]', 'dy[µm]', 'r[µm]', 'abs error x','abs error y','abs error r'});
% Display results in table

%mean of r
meanR = mean(r, 'omitnan');
fprintf(resultfile,"------------------------------------------------\n");
fprintf(resultfile,'Mean motion radius: %.2f μm\n', meanR);
%standard deviationof the motion radius
stdR = std(r, 'omitnan');
fprintf(resultfile,'Standard deviation of motion radius: %.2f μm\n', stdR);

%mean square error in the motion radius
% Calculate mean square error
mseR = mean((r - meanR).^2, 'omitnan');
fprintf(resultfile,'Mean square error of motion radius: %.2f μm²\n', mseR);
% Save the motion data to a CSV file for further analysis
writetable(motionData, '~/Documents/sensor fusion results/csv files/Linear_Move_F12-5_result.csv');
% linear regration equation for the motion radius

% Close the figure windows after processing
% Perform x^5 regeration on the motion radius
% Fit a polynomial of degree 5 to the motion radius data
p = polyfit((1:n-1)', r, 5);
fprintf(resultfile, 'Polynomial coefficients for motion radius: ');
fprintf(resultfile, '%.4f ', p);
fprintf(resultfile, '\n');
% Plot the polynomial fit for the motion radius
xFit = linspace(1, n-1, 100);
yFit = polyval(p, xFit);
figure;
plot((1:n-1)', r, 'o', xFit, yFit, '-r');
xlabel('Frame');
ylabel('Motion Radius (μm)');
title('Motion Radius and Polynomial Fit');
legend('Data', 'Polynomial Fit');
%close all;
fclose(resultfile);
appenddata = table(n+1,"Mean",meanR,"standard deviation",stdR',"mea of abs error in r",mean_abs_errorR,"Random error in r",rand_Error);
writetable(appenddata, '~/Documents/sensor fusion results/csv files/Linear_Move_F12-5_result.csv','WriteMode','append');
close("all");
%figure
absR = meanR-r;
% disp(meanR)
% fprintf("meanR: \t%f\n",absR);
theta = atan2(dy,dx);
polarscatter(theta,r);
rlim([0,(max(r) + 0.2)]);
disp(max(r))
end
