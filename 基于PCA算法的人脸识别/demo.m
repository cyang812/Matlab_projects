TrainDatabasePath = ('F:\matlab\PCA\TrainDatabase')
T = CreateDatabase(TrainDatabasePath);

%img = imread('F:\matlab\PCA\TrainDatabase\1.jpg')
%img = rgb2gray(img)

m = mean(T,2); % Computing the average face image m = (1/P)*sum(Tj's)    (j = 1 : P)
Train_Number = size(T,2);

%s = [79,225,113,120,106,113,160,162,55,53,126,128,79,74,119,121,103,102,141,138,147,149];
%m1 = mean(s) 

A = [];  
for i = 1 : Train_Number
    temp = double(T(:,i)) - m; % Computing the difference image for each image in the training set Ai = Ti - m
    A = [A temp]; % Merging all centered images
end

L = A'*A; % L is the surrogate of covariance matrix C=A*A'.计算协方差矩阵
[V D] = eig(L); % Diagonal elements of D are the eigenvalues for both L=A'*A and C=A*A'.求特征向量和特征值

L_eig_vec = [];
for i = 1 : size(V,2) 
    if( D(i,i)>1 )
        L_eig_vec = [L_eig_vec V(:,i)];
    end
end

Eigenfaces = A * L_eig_vec; % A: centered image vectors