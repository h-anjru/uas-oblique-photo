function missionSim(endlap,sidelap,flying_height,tilt)
%MISSIONSIM
%   
%  

% camera parameters (Canon EOS-Rebel SL1)
f = 24; format_y = 22.3; format_x = 14.9; pix = 0.0043; % [mm]
fov = 2*atan2d(format_x,2*f);
f_pix = f/pix;

fprintf('\nf = %i mm\n\n',f);

% calculate and plot GSD
m = round(format_x/pix,0);
p = 1:m;
mu_(p) = atan((p-1-m/2)/f_pix);
mu(p) = atan((p-m/2)/f_pix);
t = deg2rad(tilt);
gsd(p) = flying_height*(tan(t+mu(p)) - tan(t+mu_(p)));
% plot(p,gsd,'LineWidth',3);
fprintf('GSD in direction of flight:\n')
fprintf('min = %3.1f cm\n',100*min(gsd))
fprintf('med = %3.1f cm\n',100*median(gsd))
fprintf('max = %3.1f cm\n',100*max(gsd))

% calculate airbase B and flight line spacing W
B = flying_height*(tand(tilt + fov/2) - tand(tilt - fov/2))*(1 - endlap);
B = abs(B);
W = flying_height*(format_y/(f*cosd(tilt)))*(1 - sidelap);
fprintf('\nAirbase = %7.2f m\nAirwidth = %6.2f m\n',B,W)

% set up EOPs; first, where optical axis intersects ground
x = 100; y = 100;
X1 = linspace(x,x + 3*B,4); X2 = X1 + B/2;
Y1 = [y,y + 2*W];Y2 = y + W;

% offset for tilt 
offset = flying_height*tand(tilt);
X1o = X1 - offset; X2o = X2 + offset;

[XL,YL,ZL] = meshgrid(X1o,Y1,flying_height); eop = [XL(:) YL(:) ZL(:)];
[XL,YL,ZL] = meshgrid(X2o,Y2,flying_height); eop = [eop;XL(:) YL(:) ZL(:)];

% get EOPs in serpentine order
eop = sortrows(eop,2);
eop(5:8,:) = sortrows(eop(5:8,:),1,'descend');

% +/-tilt is a -/+y rotation in this case
eop = [eop zeros(size(eop)) f_pix*ones(size(eop,1),1)];
eop(:,5) = -tilt; eop(5:8,5) = tilt;

% point cloud for the testing
extend = -flying_height/(0.2*f);
xv = linspace(X2(1) - extend, X1(4) + extend, 3);
yv = linspace(Y1(1) - extend, Y1(2) + extend, 3);
zv = linspace(0,20,3);
[X,Y,Z] = meshgrid(xv,yv,zv); pts = [X(:) Y(:) Z(:)];
names = 101:101+size(pts,1)-1;
pts = [names' pts];

% cell array to store image matrices
images = cell(size(eop,1),1);

% call eop iteratively, send to backproject, store results in cell array
for ii = 1:size(eop,1)
    images{ii} = backproject(pts,eop(ii,:),0);
end

% write results to bun2013.exe dat file
outfile = ['bun_e' num2str(endlap*100) ...
    '_s' num2str(sidelap*100) ...
    '_h' num2str(flying_height) ...
    '_f' num2str(f) ...
    '_t' num2str(tilt) '.dat'];
fid = fopen(outfile,'wt');

% focal length
fprintf(fid,'%8.3f\n',f_pix);

% 'initial approximations' of EOPs of photos
str = 'A Image%i %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n';
for ii = 1:length(images)
    fprintf(fid,str,ii,eop(ii,4:6),eop(ii,1:3));
end
fprintf(fid,'#\n');

% control points and 'initial approximations' of obj space coords
control = [1 3 5 7 9];  % which pts are control points
str1 = '3 %i %8.3f %8.3f %8.3f   0.005   0.005   0.005\n';
str2 = 'A %i %8.3f %8.3f %8.3f\n';
for jj = 1:length(pts)
    if any(jj == control)
        fprintf(fid,str1,pts(jj,:));
    else
        fprintf(fid,str2,pts(jj,:));
    end
end
fprintf(fid,'#\n0.5 0.5\n');

% image points
str = 'D %i %8.3f %8.3f\n';
for kk = 1:length(images)
    image = images{kk};
    fprintf(fid,'Image%i\n',kk);  % image name
    fprintf(fid,str,image);
    fprintf(fid,'#\n');
end
fprintf(fid,'#\n');
fclose(fid);

%% visualize
% individual images
% for pp = 1:length(images)
%     figure(pp)
%     hold on;
%     grid on; axis equal;
%     xlabel 'x [pix]';ylabel 'y [pix]';
%     xlim([-format_x/pix/2 format_x/pix/2])
%     ylim([-format_y/pix/2 format_y/pix/2])
%     X = images{pp};
%     fprintf('Plotting image %i: ',pp)
%     fprintf('%i %i\n',size(X))
%     plot(X(2,:),X(3,:),'k.');
% end
% hold off;
% initialize object space figure
fig = figure;
hold on;
title(['flying height = ' num2str(flying_height) ...
    'm, tilt = ' num2str(tilt) '°'])
grid on; zlim([0 flying_height+20]);
ax = gca; ax.GridColor = [0 0 0]; ax.Projection = 'perspective';
daspect([1 1 1])
xlabel 'X [m]';ylabel 'Y [m]';zlabel 'Z [m]';
plot3(pts(:,2),pts(:,3),pts(:,4),'.') % ground points
% intersections of optical axes w/ ground
[Xg,Yg] = meshgrid(X1,Y1); grnd = [Xg(:) Yg(:)];
[Xg,Yg] = meshgrid(X2,Y2); grnd = [grnd;Xg(:) Yg(:)];
plot(grnd(:,1),grnd(:,2),'k.','MarkerSize',2)
% exposure stations and footprints
plane_normal = [0 0 1]'; plane_normal = repmat(plane_normal,1,5);
plane_point = [0 0 0]'; plane_point = repmat(plane_point,1,5);
x_ = 0.5*format_x; y_ = 0.5*format_y;
v2 = [-x_ +y_ -f]'; v1 = [+x_ +y_ -f]';
v3 = [-x_ -y_ -f]'; v4 = [+x_ -y_ -f]';
vx = [+x_ 0 0]'; vy = [0 +y_ 0]'; vz = [0 0 +f]';
for nn = 1:length(eop)
    scale = 500;
    R = makehgtform('yrotate',deg2rad(eop(nn,5))); T = eop(nn,1:3);
    % axes and format
    ax = R(1:3,1:3)*[vx vy vz]; ax = scale*0.001*ax;
    plot3([T(1) ax(1,1)+T(1)],[T(2) ax(2,1)+T(2)],...
        [T(3) ax(3,1)+T(3)],'r','LineWidth',2)
    plot3([T(1) ax(1,2)+T(1)],[T(2) ax(2,2)+T(2)],...
        [T(3) ax(3,2)+T(3)],'g','LineWidth',2)
    plot3([T(1) ax(1,3)+T(1)],[T(2) ax(2,3)+T(2)],...
        [T(3) ax(3,3)+T(3)],'b','LineWidth',2)
    fo = R(1:3,1:3)*([v1 v2 v3 v4 v1]+[0;0;f]); fo = scale*0.001*fo;
    plot3(fo(1,:)+T(1),fo(2,:)+T(2),fo(3,:)+T(3),'k-','LineWidth',2)
    % footprints
    v = R(1:3,1:3)*[v1 v2 v3 v4 v1];
    % conditional for high obliques: check z-orientations
    if v(3,1) < 0 && v(3,2) < 0 && v(3,3) < 0 && v(3,4) < 0
        % do nothing
    elseif v(3,1) > 0 && v(3,2) < 0 && v(3,3) < 0 && v(3,4) > 0
        v = R(1:3,1:3)*[vy+[0 0 -f]' v2 v3 -vy+[0 0 -f]'];
    else
        v = R(1:3,1:3)*[vy+[0 0 -f]' v1 v4 -vy+[0 0 -f]'];
    end
    for mm = 1:length(v)
        d = dot((plane_point(:,mm)-T'),plane_normal(:,mm))/...
            (dot(v(:,mm),plane_normal(:,mm)));
        points(:,mm) = d*v(:,mm) + T';
    end
    plot(points(1,:),points(2,:),'k:','LineWidth',2)
end
% UI control for scaling 
hold off;
view([-30 30])
% figname = ['bun_e' num2str(endlap*100) ...
%     '_s' num2str(sidelap*100) ...
%     '_h' num2str(flying_height) ...
%     '_f' num2str(f) ...
%     '_t' num2str(tilt) '.png'];
% saveas(fig,figname)

% assignin('base','pts',pts)
% assignin('base','eop',eop)
% assignin('base','B',B)
% assignin('base','W',W)
% assignin('base','images',images)

end