 function img = warp_t(inputimg, M)
 img=zeros(50,50);
  [x,y] = meshgrid(1:50,1:50);
  x = x(:); y = y(:); % Vectorize everything
 xwarp = round((M(1,1).*x+M(1,2).*y+M(1,3))./(M(3,1).*x+M(3,2).*y+M(3,3)));
 ywarp = round((M(2,1).*x+M(2,2).*y+M(2,3))./(M(3,1).*x+M(3,2).*y+M(3,3)));
 for j=1:1:2500
     img(x(j),y(j))=inputimg(xwarp(j),ywarp(j));
 end
 