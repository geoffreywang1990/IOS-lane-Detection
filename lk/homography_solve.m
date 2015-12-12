function v = homography_solve(pin, pout)
% HOMOGRAPHY_SOLVE finds a homography from point pairs
%   V = HOMOGRAPHY_SOLVE(PIN, POUT) takes a 2xN matrix of input vectors and
%   a 2xN matrix of output vectors, and returns the homogeneous
%   transformation matrix that maps the inputs to the outputs, to some
%   approximation if there is noise.
%
%   This uses the SVD method of
%   http://www.robots.ox.ac.uk/%7Evgg/presentations/bmvc97/criminispaper/node3.html
% David Young, University of Sussex, February 2008
W=pin;
X=pout;   
    for i=1:1:4
         a = W(1,i);
         b = W(2,i);
         x = X(1,i);
         y = X(2,i);
        A(2*i-1:2*i,:) = [0, 0, 0, -a, -b, -1, y*a, y*b, y;
               a, b, 1, 0, 0, 0, -x*a, -x*b, -x];
        
    end;
   [U,S,V] = svd(A);        
    Phi =V(:,9);
   
   H=reshape(Phi,3,3);
   H = H./H(3,3);
   v = H';
   


end