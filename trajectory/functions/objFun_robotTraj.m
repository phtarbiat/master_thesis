function objFunValue = objFun_robotTraj(paramStruct,ab,timeVec)
% Objective function for finding the optimal trajectory for identifying the
% base parameters of the robot
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Load kinematic parameters
g = paramStruct.g;
i_g = paramStruct.i_g;
r_P1P2 = paramStruct.r_P1P2;
r_P2P3 = paramStruct.r_P2P3;
r_P3P4 = paramStruct.r_P3P4;


% Calculate observation matrix
Y = YFun_fourierSeries(g,i_g,r_P1P2,r_P2P3,r_P3P4,ab,timeVec(1));

nRow = size(Y,1);
nCol = size(Y,2);
nSteps = length(timeVec);

W = zeros(nRow*nSteps,nCol);

for iStep = 1:nSteps
    posRow = ((iStep - 1)*nRow + 1):(iStep*nRow);
    
    W(posRow,:) = YFun_fourierSeries(g,i_g,r_P1P2,r_P2P3,r_P3P4,ab,timeVec(iStep));
end


% Calculate condition number
objFunValue = cond(W);