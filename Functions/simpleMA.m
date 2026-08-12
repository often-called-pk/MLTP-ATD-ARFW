function s_out = simpleMA(s_in,N,M)
%Calculate moving average:
%   -s_in: input signal
%   -N: number of intervals used for the moving average
%   -M: number of times to apply the moving average

if nargin == 2
    M = 1;
end
s_out = s_in;
for i=1:M
    coeff = ones(1,round(N))/(N); 
    s_out = filter(coeff,1,s_out); %calculate moving average
    s_out = circshift(s_out, -round(N/2)); %shift to avoid characteristic delay of moving averages
end

end