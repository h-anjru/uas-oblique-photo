function missionSim(endlap, sidelap, flying_height, tilt)
%MISSIONSIM Creates a small simulated photo collection mission for testing 
%  bundle adjustment results with BUN2013.exe.
%
%  Call: missionSim(endlap, sidelap, flying_height, tilt) 
%
%         endlap: endlap from photo to photo (0.0-1.0)
%        sidelap: sidelap from flight line to flight line (0.0-1.0)
%  flying_height: height above ground (meters)
%           tilt: angle of tilt of camera from nadir (deg)
% 
%         output: ASCII file (.dat) which is formatted for input into a
%                 bundle adjustment software, BUN2013.exe, written by Bon 
%                 Dewitt.
% 
%  Reference: Wolf, Dewitt, Wilkinson. "Elements of Photogrammetry 4th ed."
%
%  Original equations for GSD, B, and W by Lassiter (2017).


    % --------------------------------------------
    % camera parameters [mm] (Canon EOS-Rebel SL1)
    % --------------------------------------------
    f = 24;
    format_x = 22.3;
    format_y = 14.9;
    pix = 0.0043;

    % calculate field of view
    fov = 2*atan2d(format_y, 2 * f);

    % convert focal length from mm to pixels
    format = [format_x, format_y] ./ pix;
    f_pix = f / pix;

    % calculate ground sample distance (GSD)
    m = round(format_y / pix, 0);
    p = 1:m;
    mu_(p) = atan((p - 1 - m/2) / f_pix);
    mu(p) = atan((p - m / 2) / f_pix);
    t = deg2rad(tilt);
    gsd(p) = flying_height*(tan(t+mu(p)) - tan(t+mu_(p)));

%     % ground sample distance plot fodder
%     fig_GSD = figure(0);
%     fig_GSD.Position = [100 100 100 100];
% 
%     plot_GSD = plot(p, gsd);
%     plot_GSD.LineWidth = 3;
% 
%     hold on;
% 
%     hold off;

    % calculate airbase B and flight line spacing W
    B = flying_height * ...
        (tand(tilt + fov / 2) - tand(tilt - fov / 2)) * ...
        (1 - endlap);
    B = abs(B);

    W = flying_height * ...
        (format_x / (f * cosd(tilt))) * ...
        (1 - sidelap);

    % print quick mission statistics in command window
    fprintf('\n');
    fprintf('f = %i mm\n', f);
    fprintf('\n');
    fprintf('GSD in direction of flight:\n');
    fprintf('min = %3.1f cm\n', 100 * min(gsd));
    fprintf('med = %3.1f cm\n', 100 * median(gsd));
    fprintf('max = %3.1f cm\n', 100 * max(gsd));
    fprintf('\n');
    fprintf('Airbase = %7.2f m\n', B);
    fprintf('Airwidth = %6.2f m\n', W);


    % ---------------------------
    % establish exposure stations
    % ---------------------------
    % assume the first exposore station's optical axis intersects h = 0 at
    % point (x,y)
    x = 100; y = 100;

    % using B and W, form a three-line pattern (four exposures per line).
    % initial direction of travel: north (+Y)
    XL_initial = linspace(x, x + 2 * W, 3);
    YL_initial = linspace(y, y + 3 * B, 4);
    
    [XL, YL, ZL] = meshgrid(XL_initial, YL_initial, flying_height);
    cam = [XL(:), YL(:), ZL(:)];

    % sort exposure stations into serpentine order
    offset = flying_height * tand(tilt);
    cam(5:8, :) = sortrows(cam(5:8, :), 2, 'descend');

    % add an offset for tilt of camera
    cam(1:4, 2) = cam(1:4, 2) - offset;
    cam(5:8, 2) = cam(5:8, 2) + offset;
    cam(9:12, 2) = cam(1:4, 2);

    % cam is an array: XL, YL, ZL, o, p, k, f
    % tilt is an omega (o) rotation in this case
    cam = [cam, zeros(size(cam)), f_pix * ones(size(cam, 1), 1)];
    cam(:, 4) = tilt;
    cam(5:8, 6) = 180;  % traveling south

    assignin('caller', 'cam', cam);

    % create 3D point array in object space for testing
    target_height = 20;

    extend = flying_height / (0.2 * f);

    xv = linspace(XL_initial(2) - extend, XL_initial(2) + extend, 3);
    yv = linspace(YL_initial(2) - extend, YL_initial(3) + extend, 3);
    zv = linspace(0, target_height, 3);

    [X, Y, Z] = meshgrid(xv, yv, zv);
    pts = [X(:), Y(:), Z(:)];

    names = 101:101 + size(pts, 1) - 1;
    pts = [names', pts];

    assignin('caller', 'pts', pts);

    % cell array to store image matrices
    images = cell(size(cam, 1), 1);

    % call camera stations, send to backproject
    for ii = 1:size(cam, 1)
        images{ii} = backproject(pts, cam(ii, :), 0.5, format);
    end

    % write results to bun2013.exe dat file
    outfile = [
        'bun_e' num2str(endlap * 100), ...
        '_s' num2str(sidelap * 100), ...
        '_h' num2str(flying_height), ...
        '_f' num2str(f), ...
        '_t' num2str(tilt), ...
        '.dat'
    ];
    fid = fopen(outfile, 'wt');

    % write focal length
    fprintf(fid, '%8.3f\n', f_pix);

    % write 'initial approximations' of camera stations
    str = 'A Image%i %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f\n';
    for ii = 1:length(images)
        fprintf(fid, str, ii, cam(ii, 4:6), cam(ii, 1:3));
    end
    fprintf(fid,'#\n');

    % write GCPs and 'initial approximations' of obj space coords
    control = [1, 3, 5, 7, 9];  % indices of GCPs (X-pattern)
    str1 = '3 %i %8.3f %8.3f %8.3f   0.005   0.005   0.005\n';
    str2 = 'A %i %8.3f %8.3f %8.3f\n';
    for jj = 1:length(pts)
        if any(jj == control)
            fprintf(fid, str1, pts(jj, :));
        else
            fprintf(fid, str2, pts(jj, :));
        end
    end
    fprintf(fid, '#\n0.5 0.5\n');

    % write image points to .dat file
    str = 'D %i %8.3f %8.3f\n';
    for kk = 1:length(images)
        image = images{kk};
        fprintf(fid, 'Image%i\n', kk);  % image name
        fprintf(fid, str, image);
        fprintf(fid, '#\n');
    end
    fprintf(fid, '#\n');
    fclose(fid);


%     % plot fodder for individual images
%     for pp = 1:length(images)
%         fig_img = figure(pp);
%         fig_img.Position = [300 300 300 300];     
%         hold on;
%         grid on;
%         axis equal;
% 
%         xlabel 'x [pix]';
%         ylabel 'y [pix]';
% 
%         xlim([-format_x / pix / 2, format_x / pix / 2])
%         ylim([-format_y / pix / 2, format_y / pix / 2])
% 
%         X = images{pp};
% 
%         fprintf('Plotting image %i: ', pp)
%         fprintf('%i %i\n', size(X))
%         plot(X(2, :), X(3, :), 'k.');
%     end
%     
%     hold off;


    % initialize object space figure
    fig_world = figure;
    hold on;

    title([
        'flying height = ' num2str(flying_height), ...
        'm, tilt = ' num2str(tilt) '°'
    ])

    grid on;

    ax = gca;
    ax.GridColor = [0, 0, 0];
    ax.Projection = 'perspective';

    xlabel 'X [m]';
    ylabel 'Y [m]';
    zlabel 'Z [m]';

    ax.ZLim = [0, flying_height + 20];

    daspect([1 1 1]);
    view([-30 30])

    % plot object space array
    plot_array = plot3(pts(:, 2), pts(:, 3), pts(:, 4));
    plot_array.LineStyle = 'none';
    plot_array.Marker = '.';
    plot_array.MarkerSize = 8;
    plot_array.Color = 'b';

    % intersections of optical axes w/ ground
    [Xg, Yg] = meshgrid(XL_initial, YL_initial);
    grnd = [Xg(:) Yg(:)];

    plot_opt = plot(grnd(:, 1), grnd(:, 2));
    plot_opt.LineStyle = 'none';
    plot_opt.Marker = '+';
    plot_opt.MarkerSize = 8;
    plot_opt.Color = 'm';

    % camera stations and footprints
    plane_normal = [0 0 1]';
    plane_normal = repmat(plane_normal, 1, 5);
    plane_point = [0 0 0]';
    plane_point = repmat(plane_point, 1, 5);

    x_ = 0.5 * format_x;
    y_ = 0.5 * format_y;

    v1 = [+x_ +y_ -f]';
    v2 = [-x_ +y_ -f]'; 
    v3 = [-x_ -y_ -f]';
    v4 = [+x_ -y_ -f]';
    vx = [+x_ 0 0]';
    vy = [0 +y_ 0]';
    vz = [0 0 +f]';

    for nn = 1:length(cam)
        % scale the exposure stations for plotting
        scale = 500;
        
        % direct rotation matrix to rotate frustra
        Ro = makehgtform('xrotate', deg2rad(cam(nn, 4)));
        Rk = makehgtform('zrotate', deg2rad(cam(nn, 6)));
        R = Rk * Ro;

        T = cam(nn, 1:3);

        % axes and format
        ax = R(1:3, 1:3) * [vx vy vz];
        ax = scale * 0.001 * ax;

        % image x-axes
        plot3( ...
            [T(1), ax(1, 1) + T(1)], ...
            [T(2), ax(2, 1) + T(2)], ...
            [T(3), ax(3, 1) + T(3)], ...
            'r', 'LineWidth', 2 ...
        );

        % image y-axes
        plot3( ...
            [T(1), ax(1, 2) + T(1)], ...
            [T(2), ax(2, 2) + T(2)], ...
            [T(3), ax(3, 2) + T(3)], ...
            'g', 'LineWidth', 2 ...
        );

        % image z-axes
        plot3( ...
            [T(1), ax(1, 3) + T(1)], ...
            [T(2), ax(2, 3) + T(2)], ...
            [T(3), ax(3, 3) + T(3)], ...
            'b', 'LineWidth', 2 ...
        );

        % image format
        fo = R(1:3, 1:3) * ([v1, v2, v3, v4, v1] + [0; 0; f]);
        fo = scale * 0.001 * fo;  % scale from mm to m

        plot3( ...
            fo(1, :) + T(1), ...
            fo(2, :) + T(2), ...
            fo(3, :) + T(3), ...
            'k-', 'LineWidth', 2 ...
        );

        % footprints
        v = R(1:3, 1:3) * [v1, v2, v3, v4, v1];

        % conditional for high obliques: check z-orientations
        if v(3, 1) < 0 && v(3, 2) < 0 && v(3, 3) < 0 && v(3, 4) < 0
            % do nothing
        elseif v(3, 1) > 0 && v(3, 2) < 0 && v(3, 3) < 0 && v(3, 4) > 0
            v = R(1:3, 1:3) * [vy + [0 0 -f]' v2 v3 -vy + [0 0 -f]'];
        else
            v = R(1:3, 1:3) * [vy + [0 0 -f]' v1 v4 -vy+[0 0 -f]'];
        end

        for mm = 1:length(v)
            d = dot((plane_point(:, mm) - T'), plane_normal(:, mm)) /...
                (dot(v(:, mm), plane_normal(:, mm)));
            points(:, mm) = d * v(:, mm) + T';
        end

        plot_footprints = plot(points(1, :), points(2, :));
        plot_footprints.LineStyle = ':';
        plot_footprints.LineWidth = 2;
        plot_footprints.Color = 'k';
    end

    hold off;

end
