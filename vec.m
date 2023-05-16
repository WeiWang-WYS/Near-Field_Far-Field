function y = vec(x)

[row col] = size(x);

y = reshape(x, row*col, 1);