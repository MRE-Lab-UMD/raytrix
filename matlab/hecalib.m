clc
clear all
close all
format long

% points in the raytrix frame
% rosrun rviz rviz and then echo the topic /rviz/select
% and click on the 4 corner in the ABCD order
%  A    D
%        
%  B    C
Pc = [
-0.00500585930422 -0.00507031241432 0.00485546886921 0.00479101575911
-0.00485546886921 0.00476953107864 0.00483398418874 -0.00479101575911 
  0.00322365155444 0.00203225552104 0.00222107232548 0.00356238195673 ];

%Pc = [
%    0.439453125 0.455078125 -0.443359375 -0.45703125
%    -0.42578125 0.4296875 0.4296875 -0.427734375
%    0.682559192181 0.82813668251 0.845786333084 0.723961889744
%     ];

%Pc = [ ...
%-0.435546875, -0.458984375, 0.439453125, 0.451171875; ...
%-0.376953125, 0.396484375, 0.3828125, -0.396484375; ...
%0.399122655392, 0.153190851212, 0.143150806427, 0.363030970097 ...
%];
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              

% points in the robot frame
% put the rod on and measure the XYZ coordinates
% in the ABCD order (same order than for the camera)
Pr = [
19.83      19.95   9.47   9.46     
  124.12    114.27 114.04  124.08
1523.99	 1524.24  1523.86 1523.54
     ] / 1000;

%Pr = [
%    10.95 12.55 2.12 1.22
%    151.50 141.18 140.30 150.27
%    1526.57 1526.74 1526.69 1526.69
%     ] / 1000;

%Pr = [ ...
%8.11, 8.33, -1.35, -1.80; ...
%150.81, 140.81, 139.99, 149.99; ...
%46+[1475.17, 1475.22, 1475.22, 1479.99]/1000 ...
%];

A = zeros( [4,3] );

% Here's the deal, the raytrix returns coordinate in an unknown
% coordinate frame and with an unknown scale. The raytrix node
% scales the X and Y to be within the [-0.5, 0.5] intervals and the
% Z to be within [0,1]

% So we must first figure the scale along each axis. If the square
% pattern has a side length l then the norm of the difference
% between each consecutive vertices should be l.

% To find the scales along each axis we solve the following
% overdetermined system of equations
%
%  sqrt(  (a*x1 - a*x2)^2 + (b*y1 - b*y2)^2 + (c*z1 - c*z2)^2 ) = l
%  sqrt(  (a*x2 - a*x3)^2 + (b*y2 - b*y3)^2 + (c*z2 - c*z3)^2 ) = l 
%  sqrt(  (a*x3 - a*x4)^2 + (b*y3 - b*y4)^2 + (c*z3 - c*z4)^2 ) = l 
%  sqrt(  (a*x4 - a*x1)^2 + (b*y4 - b*y1)^2 + (c*z4 - c*z1)^2 ) = l 

% Linearizing the system of equations we get
%
% [ (x1-x2)^2    (y1-y2)^2    (z1-z2)^2 ] [ a^2 ]   [ l^2 ]
% [ (x2-x3)^2    (y2-y3)^2    (z2-z3)^2 ] [ b^2 ] = [ l^2 ]
% [ (x3-x4)^2    (y3-y4)^2    (z3-z4)^2 ] [ c^2 ]   [ l^2 ]
% [ (x4-x1)^2    (y4-y1)^2    (z4-z1)^2 ]           [ l^2 ]


A(1,1) = (Pc(1,1)-Pc(1,2))^2;
A(1,2) = (Pc(2,1)-Pc(2,2))^2;
A(1,3) = (Pc(3,1)-Pc(3,2))^2;

A(2,1) = (Pc(1,2)-Pc(1,3))^2;
A(2,2) = (Pc(2,2)-Pc(2,3))^2;
A(2,3) = (Pc(3,2)-Pc(3,3))^2;

A(3,1) = (Pc(1,3)-Pc(1,4))^2;
A(3,2) = (Pc(2,3)-Pc(2,4))^2;
A(3,3) = (Pc(3,3)-Pc(3,4))^2;

A(4,1) = (Pc(1,4)-Pc(1,1))^2;
A(4,2) = (Pc(2,4)-Pc(2,1))^2;
A(4,3) = (Pc(3,4)-Pc(3,1))^2;

l = 0.01; % assuming we have a 10 mm x 10 mm (0,010m) square
b = [ l^2; l^2; l^2; l^2 ];

abc = sqrt( A\b );

save( 'raytrixscales.txt', 'abc', '-ascii' );

% validate
Pc = [ abc(1)*Pc(1,:) ; abc(2)*Pc(2,:); abc(3)*Pc(3,:) ];

% this should be small
L = [ norm( Pc(:,1) - Pc(:,2) ) - l
      norm( Pc(:,2) - Pc(:,3) ) - l
      norm( Pc(:,3) - Pc(:,4) ) - l
      norm( Pc(:,4) - Pc(:,1) ) - l ]'


Rtrc = rigid_transform_3D( Pc', Pr' )
Rrc = Rtrc(1:3,1:3);
trc = Rtrc(1:3,4)'
q = Quaternion( Rrc )

Prc = Rtrc * [ Pc; ones( 1, 4 ) ];
% Find the error
err = Prc(1:3,:) - Pr;
err = err .* err; % square
err = sum(err(:));

% This should be small
rmse = sqrt(err/4)

% 
fid = fopen( 'Rtrc.txt', 'w' );
fprintf( fid, ['#Cut and paste the following line in the hecalib.sh ' ...
               'script\n'] );
fprintf( fid, ...
         'rosrun tf static_transform_publisher %f %f %f %f %f %f %f "LWR_0" "raytrix" 0.01\n', ...
         trc(1), trc(2), trc(3), q.v(1), q.v(2), q.v(3), q.s );
fclose( fid );

                                                                 
