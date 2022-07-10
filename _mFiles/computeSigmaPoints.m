%   Compute set of sigma points function
function [X,W,mX,PX] = computeSigmaPoints(mx,Pxx,method,kappa,alpha,beta)
if alpha==0
	error('alpha cannot be 0!');
end
sqrtFcn = @chol;
% Compute number of samples
n = size(mx,1);
% Compute sigma points
switch method	
    case 'symmetric'	
      % Compute unscaled sigma points
      X(:,1) = mx;
      matrixSq = sqrtFcn((n+kappa)*Pxx)';
      X(:,2:n+1) = repmat(mx,1,n) + matrixSq;
      X(:,n+2:2*n+1) = repmat(mx,1,n) - matrixSq;
      % Compute weights
      W(1) = kappa/(n+kappa);
      W(2:2*n+1) = .5/(n+kappa);
      
	case 'spherical'		
      % Compute weights
      W(1) = kappa;
      W(2:n+2) = (1-W(1))/(n+1);
      % Initialize vector sequence
      X(1,1) = 0;
      X(1,2) = -1/sqrt(2*W(2));
      X(1,3) = 1/sqrt(2*W(2));
      % Construct vector sequence
      XOld = X;
      for j=2:n
        X = zeros(j,j+2);
        for i=0
            X(:,i+1) = [XOld(:,i+1); 0];
        end
        for i=1:j
            X(:,i+1) = [XOld(:,i+1); -1/sqrt(j*(j+1)*W(2))];
        end
        for i=j+1
            X(:,i+1) = [zeros(j-1,1); j/sqrt(j*(j+1)*W(2))];
        end
        XOld = X;
      end
      % Transform to correct mean and covariance
      X = repmat(mx,1,n+2) + sqrtFcn(Pxx)'*X;
end

% Scale the sigma points
p = size(X,2);
X = repmat(X(:,1),1,p) + alpha*(X-repmat(X(:,1),1,p));
% Compute weights
W(1) = W(1)/alpha^2 + (1-1/alpha^2);
W(2:end) = W(2:end)/alpha^2;
% Compute weighted mean and covariance
mX = wmean(X,W);
PX = wcov(X,W,alpha,beta);

