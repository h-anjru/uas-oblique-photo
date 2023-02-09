function x = backproject(X, cam, sigma, format)
%BACKPROJECT Back project object space coordinates into two images. This
%  function assumes an ideal pinhole camera. Noise is added to the back-
%  projected image coordinates.
%  
%  Call: [x1, x2, noise] = backproject(X, cam1, cam2, sigma)
%  
%       X: [n, n x 3] matrix of n names, object space points [name, X, Y, Z]
%     cam: row vector of EOPs for camera 1 [XL, YL, ZL, tilt, [unused], azimuth, f(pix)]
%   sigma: stdev of image measurements (typically 0.5 pixel)
%  format: [2 x 1] matrix of format_x and format_y (pixels)
%  
%      x: 2 x n matrix of image coordinates for cameras 1 & 2 [x1, y1, x2, y2]
%
%  Reference: Wolf, Dewitt, Wilkinson. "Elements of Photogrammetry 4th ed."
%    Appendix D-4.

    % perspective matrices
    P1 = [eye(3), zeros(3, 1)];
    P1(3,3) = -1 / cam(7);

    % rotation matrices (more in the ats or
    M_tilt = makehgtform('xrotate', deg2rad(cam(4)));
    M_azimuth = makehgtform('zrotate', deg2rad(cam(6)));
    M1 = M_tilt' * M_azimuth';

    % translation matrices
    T1 = eye(4);
    T1(1:3, 4) = -cam(1: 3)';

    % homogeneous representation of object space coordinates
    X_ = [X(:, 2:4), ones(size(X, 1), 1)];
    X_ = X_';

    % homogenous representation of collinearity equations
    W1 = P1 * M1 * T1 * X_;

    % projection onto plane z = 1
    x = bsxfun(@rdivide, W1, W1(3, :));
    x = x(1:2, :);  % row 3 omitted

    % addition of noise
    noise = randn(size(x)) .* sigma;
    x = x + noise;
    x = [X(:, 1)'; x];

    % filter out results outside of format
    x = format_filter(x, format);

end


function xout = format_filter(x_in, format_in)

    % format
    format_x = format_in(1);
    format_y = format_in(2);

    for ii = 1:size(x_in, 2)
        if abs(x_in(2, ii)) > 0.5 * format_x || abs(x_in(3, ii)) > 0.5 * format_y
%         fprintf('Removing point %i...\n', x_in(1, ii))
        x_in(:, ii) = [NaN; NaN; NaN]; 
        end
    end

    % remove NaN values and reshape to three columns
    if any(any(isnan(x_in)))
        x_in(isnan(x_in)) = []; 
        xout = reshape(x_in, 3, int8(length(x_in)) / 3);
    else
        xout = x_in;
    end

end
