% This function finds the optimal Rigid/Euclidean transform in 3D space
% It expects as input a Nx3 matrix of 3D points.
% A 4x4 transformation matrix is returned

% You can verify the correctness of the function by copying and pasting these commands:
%{

%R = orth(rand(3,3)); % random rotation matrix
%t = rand(3,1); % random translation

%n = 100; % number of points
%A = rand(n,3);
%B = R*(A') + repmat(t, 1, n);
%B = B';

%T = rigid_transform_3D(A, B);

%A2 = T * ([A ones(n,1)]');
%A2 = A2';
%A2 = A2(:,1:3); % remove the 1's added earlier

% Find the error
%err = A2 - B;
%err = err .* err; % square
%err = sum(err(:));
%rmse = sqrt(err/n);

%disp(sprintf("RMSE: %f", rmse));
%disp("If RMSE is near zero, the function is correct!");

%}

function T = rigid_transform_3D(A, B)
    
% Skipping error checking of input to reduce code clutter
    
    centroid_A = mean(A);
    centroid_B = mean(B);
    
    H = zeros(3,3);
    
    for i=1:size(A,1)
        H = H +(A(i,:) - centroid_A)' * (B(i,:) - centroid_B);
    end
    
    [U,S,V] = svd(H);
    
    R = V*U';
    if( det(R) < 0 )
        V(1:3,3) = -V(1:3,3);
        R = V*U';
    end

    C_A = eye(4,4);
    C_B = eye(4,4);
    R_new = eye(4,4);
    
    C_A(1:3, 4) = -centroid_A';
    R_new(1:3, 1:3) = R;
    C_B(1:3, 4) = centroid_B;
    
    T = C_B * R_new * C_A;
end

