rgb = imread('test.PNG'); 
img = rgb2gray(rgb); 
aaa=imresize(img,[480,640]);
roi=double(aaa(240:480,1:640));
%temp = imread('template.jpg');
row=size(roi,1);
col=size(roi,2);
temp=zeros(50,50);
temp(:,13:14)=ones(50,2)*255;
temp(:,37:38)=ones(50,2)*255;
%need to select a good region
pts = [60, 60,161, 161;
       107,  533, 107,533]; 
 
% Height and width of the template
dsize = [50,50];  
   
% Set the template points (in order that points appear in image)
tmplt_pts = [0, 0, 49,49 ; 
             0, 49, 0,49]; 

         
 M = homography_solve(tmplt_pts, pts);
 p = M2p(M);
 fb = fspecial('gaussian',[5,5], 3);
 fx = [-1,1]; 
 fy = fx'; 
 temp = double(imfilter(temp, fb));
 tempx = double(imfilter(temp,fx));
 tempy = double(imfilter(temp,fy));

 [x,y] = meshgrid(0:size(temp,2)-1,0:size(temp,1)-1);
 x = x(:); y = y(:); % Vectorize everything
 v1 = ones(size(x)); v0 = zeros(size(x));
            % Form the derivative of the warp function (as discussed in
            % class)
 J=zeros(2500,8);
 dWx = [ -x,  y, v1, v0,  v0, v0, -x.*(x),-y.*(x)]; 
 dWy = [ v0, v0, v0,  x,  -y, v1, -x.*(y), -y.*(y)];         
 J = dWx.*repmat(tempx(:),[1,8]) + dWy.*repmat(tempy(:),[1,8]);   
 R = inv(J'*J)*J'; 
  for n = 1:10                 
                    warpM=p2M(p);
                    I_p = double(I_p);
                    figure;
                    imshow(I_p);
                    
                    fprintf('Waiting for a key press....\n',n); 
                    pause; % Wait for the key pressM

                diff = I_p(:) - temp(:); 

                dp = -R*diff(:); 
                dm=p2M(dp);
                M=p2M(p);
                M=M*inv(dm);
                p=M2p(M)
  end
 
 