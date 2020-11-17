function [dis_dot_line] = dot_line(line_k,line_b,array_o)
%DOT_LINE 点到线之间的距离
%   此处显示详细说明
if line_k==inf||line_k==-inf
    dis_dot_line=abs(array_o(2)-line_b);
else
    molecule=(line_k*array_o(1)-array_o(2)+line_b);
    dis_dot_line=abs(molecule/sqrt(power(line_k,2)+1));
end
end

