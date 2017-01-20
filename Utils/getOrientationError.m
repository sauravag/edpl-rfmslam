function err = getOrientationError(Rtrue, Rmle, N)

err = zeros(1,N);

for i=1:N
    deltaR = Rtrue(:,:,i)*Rmle(:,:,i)';
    y = deltaR(2,1);
    x = deltaR(1,1);
    err(i) = atan2d(y,x);
end

end