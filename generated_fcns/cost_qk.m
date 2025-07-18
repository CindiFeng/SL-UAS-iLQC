function cs_x = cost_qk(t,in2,in3)
%COST_QK
%    CS_X = COST_QK(T,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 25.1.
%    18-Jul-2025 18:07:22

x1 = in2(1,:);
x2 = in2(2,:);
x3 = in2(3,:);
x4 = in2(4,:);
x5 = in2(5,:);
x6 = in2(6,:);
x7 = in2(7,:);
x8 = in2(8,:);
x9 = in2(9,:);
x10 = in2(10,:);
t2 = t-5.0;
t3 = t2.^2;
t4 = t3.*(3.0./2.0);
t5 = -t4;
t6 = exp(t5);
mt1 = [x1./1.0e+4;x2./1.0e+4;x3./1.0e+4;x4./1.0e+4;x5./1.0e+4;x6./5.0e+4;x7./5.0e+4;x8./5.0e+5+(t6.*(x8.*2.072964896828013e+2-3.109447345242019e+2))./1.0e+2-6.0e-6];
mt2 = [x9./5.0e+5+(t6.*(x9.*2.072964896828013e+2-1.036482448414006e+2))./1.0e+2;x10./5.0e+5+(t6.*(x10.*2.072964896828013e+2+6.218894690484039e+2))./1.0e+2+8.0e-6];
cs_x = [mt1;mt2];
end
