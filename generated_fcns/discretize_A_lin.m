function out1 = discretize_A_lin(in1,T_k)
%discretize_A_lin
%    OUT1 = discretize_A_lin(IN1,T_k)

%    This function was generated by the Symbolic Math Toolbox version 25.1.
%    18-Jul-2025 18:07:21

Alin1_1 = in1(1);
Alin1_2 = in1(11);
Alin1_3 = in1(21);
Alin1_4 = in1(31);
Alin1_5 = in1(41);
Alin1_6 = in1(51);
Alin1_7 = in1(61);
Alin1_8 = in1(71);
Alin1_9 = in1(81);
Alin2_1 = in1(2);
Alin2_2 = in1(12);
Alin2_3 = in1(22);
Alin2_4 = in1(32);
Alin2_5 = in1(42);
Alin2_6 = in1(52);
Alin2_7 = in1(62);
Alin2_8 = in1(72);
Alin2_9 = in1(82);
Alin3_1 = in1(3);
Alin3_2 = in1(13);
Alin3_3 = in1(23);
Alin3_4 = in1(33);
Alin3_5 = in1(43);
Alin3_6 = in1(53);
Alin3_7 = in1(63);
Alin3_8 = in1(73);
Alin3_9 = in1(83);
Alin4_1 = in1(4);
Alin4_2 = in1(14);
Alin4_3 = in1(24);
Alin4_4 = in1(34);
Alin4_5 = in1(44);
Alin4_6 = in1(54);
Alin4_7 = in1(64);
Alin4_8 = in1(74);
Alin4_9 = in1(84);
Alin5_1 = in1(5);
Alin5_2 = in1(15);
Alin5_3 = in1(25);
Alin5_4 = in1(35);
Alin5_5 = in1(45);
Alin5_6 = in1(55);
Alin5_7 = in1(65);
Alin5_8 = in1(75);
Alin5_9 = in1(85);
Alin6_1 = in1(6);
Alin6_2 = in1(16);
Alin6_3 = in1(26);
Alin6_4 = in1(36);
Alin6_5 = in1(46);
Alin6_6 = in1(56);
Alin6_7 = in1(66);
Alin6_8 = in1(76);
Alin6_9 = in1(86);
Alin7_1 = in1(7);
Alin7_2 = in1(17);
Alin7_3 = in1(27);
Alin7_4 = in1(37);
Alin7_5 = in1(47);
Alin7_6 = in1(57);
Alin7_7 = in1(67);
Alin7_8 = in1(77);
Alin7_9 = in1(87);
Alin8_1 = in1(8);
Alin8_2 = in1(18);
Alin8_3 = in1(28);
Alin8_4 = in1(38);
Alin8_5 = in1(48);
Alin8_6 = in1(58);
Alin8_7 = in1(68);
Alin8_8 = in1(78);
Alin8_9 = in1(88);
Alin9_1 = in1(9);
Alin9_2 = in1(19);
Alin9_3 = in1(29);
Alin9_4 = in1(39);
Alin9_5 = in1(49);
Alin9_6 = in1(59);
Alin9_7 = in1(69);
Alin9_8 = in1(79);
Alin9_9 = in1(89);
Alin10_1 = in1(10);
Alin10_2 = in1(20);
Alin10_3 = in1(30);
Alin10_4 = in1(40);
Alin10_5 = in1(50);
Alin10_6 = in1(60);
Alin10_7 = in1(70);
Alin10_8 = in1(80);
Alin10_9 = in1(90);
Alin1_10 = in1(91);
Alin2_10 = in1(92);
Alin3_10 = in1(93);
Alin4_10 = in1(94);
Alin5_10 = in1(95);
Alin6_10 = in1(96);
Alin7_10 = in1(97);
Alin8_10 = in1(98);
Alin9_10 = in1(99);
Alin10_10 = in1(100);
t2 = T_k.^2;
t3 = T_k.^3;
t4 = t2.^2;
mt1 = [(Alin1_1.*t2)./2.0+(Alin1_1.*t3)./6.0+(Alin1_1.*t4)./2.4e+1+Alin1_1.*T_k+1.0,(Alin2_1.*t2)./2.0+(Alin2_1.*t3)./6.0+(Alin2_1.*t4)./2.4e+1+Alin2_1.*T_k,(Alin3_1.*t2)./2.0+(Alin3_1.*t3)./6.0+(Alin3_1.*t4)./2.4e+1+Alin3_1.*T_k,(Alin4_1.*t2)./2.0+(Alin4_1.*t3)./6.0+(Alin4_1.*t4)./2.4e+1+Alin4_1.*T_k,(Alin5_1.*t2)./2.0+(Alin5_1.*t3)./6.0+(Alin5_1.*t4)./2.4e+1+Alin5_1.*T_k,(Alin6_1.*t2)./2.0+(Alin6_1.*t3)./6.0+(Alin6_1.*t4)./2.4e+1+Alin6_1.*T_k,(Alin7_1.*t2)./2.0+(Alin7_1.*t3)./6.0+(Alin7_1.*t4)./2.4e+1+Alin7_1.*T_k,(Alin8_1.*t2)./2.0+(Alin8_1.*t3)./6.0+(Alin8_1.*t4)./2.4e+1+Alin8_1.*T_k];
mt2 = [(Alin9_1.*t2)./2.0+(Alin9_1.*t3)./6.0+(Alin9_1.*t4)./2.4e+1+Alin9_1.*T_k,(Alin10_1.*t2)./2.0+(Alin10_1.*t3)./6.0+(Alin10_1.*t4)./2.4e+1+Alin10_1.*T_k,(Alin1_2.*t2)./2.0+(Alin1_2.*t3)./6.0+(Alin1_2.*t4)./2.4e+1+Alin1_2.*T_k,(Alin2_2.*t2)./2.0+(Alin2_2.*t3)./6.0+(Alin2_2.*t4)./2.4e+1+Alin2_2.*T_k+1.0,(Alin3_2.*t2)./2.0+(Alin3_2.*t3)./6.0+(Alin3_2.*t4)./2.4e+1+Alin3_2.*T_k,(Alin4_2.*t2)./2.0+(Alin4_2.*t3)./6.0+(Alin4_2.*t4)./2.4e+1+Alin4_2.*T_k,(Alin5_2.*t2)./2.0+(Alin5_2.*t3)./6.0+(Alin5_2.*t4)./2.4e+1+Alin5_2.*T_k,(Alin6_2.*t2)./2.0+(Alin6_2.*t3)./6.0+(Alin6_2.*t4)./2.4e+1+Alin6_2.*T_k];
mt3 = [(Alin7_2.*t2)./2.0+(Alin7_2.*t3)./6.0+(Alin7_2.*t4)./2.4e+1+Alin7_2.*T_k,(Alin8_2.*t2)./2.0+(Alin8_2.*t3)./6.0+(Alin8_2.*t4)./2.4e+1+Alin8_2.*T_k,(Alin9_2.*t2)./2.0+(Alin9_2.*t3)./6.0+(Alin9_2.*t4)./2.4e+1+Alin9_2.*T_k,(Alin10_2.*t2)./2.0+(Alin10_2.*t3)./6.0+(Alin10_2.*t4)./2.4e+1+Alin10_2.*T_k,(Alin1_3.*t2)./2.0+(Alin1_3.*t3)./6.0+(Alin1_3.*t4)./2.4e+1+Alin1_3.*T_k,(Alin2_3.*t2)./2.0+(Alin2_3.*t3)./6.0+(Alin2_3.*t4)./2.4e+1+Alin2_3.*T_k,(Alin3_3.*t2)./2.0+(Alin3_3.*t3)./6.0+(Alin3_3.*t4)./2.4e+1+Alin3_3.*T_k+1.0,(Alin4_3.*t2)./2.0+(Alin4_3.*t3)./6.0+(Alin4_3.*t4)./2.4e+1+Alin4_3.*T_k];
mt4 = [(Alin5_3.*t2)./2.0+(Alin5_3.*t3)./6.0+(Alin5_3.*t4)./2.4e+1+Alin5_3.*T_k,(Alin6_3.*t2)./2.0+(Alin6_3.*t3)./6.0+(Alin6_3.*t4)./2.4e+1+Alin6_3.*T_k,(Alin7_3.*t2)./2.0+(Alin7_3.*t3)./6.0+(Alin7_3.*t4)./2.4e+1+Alin7_3.*T_k,(Alin8_3.*t2)./2.0+(Alin8_3.*t3)./6.0+(Alin8_3.*t4)./2.4e+1+Alin8_3.*T_k,(Alin9_3.*t2)./2.0+(Alin9_3.*t3)./6.0+(Alin9_3.*t4)./2.4e+1+Alin9_3.*T_k,(Alin10_3.*t2)./2.0+(Alin10_3.*t3)./6.0+(Alin10_3.*t4)./2.4e+1+Alin10_3.*T_k,(Alin1_4.*t2)./2.0+(Alin1_4.*t3)./6.0+(Alin1_4.*t4)./2.4e+1+Alin1_4.*T_k,(Alin2_4.*t2)./2.0+(Alin2_4.*t3)./6.0+(Alin2_4.*t4)./2.4e+1+Alin2_4.*T_k];
mt5 = [(Alin3_4.*t2)./2.0+(Alin3_4.*t3)./6.0+(Alin3_4.*t4)./2.4e+1+Alin3_4.*T_k,(Alin4_4.*t2)./2.0+(Alin4_4.*t3)./6.0+(Alin4_4.*t4)./2.4e+1+Alin4_4.*T_k+1.0,(Alin5_4.*t2)./2.0+(Alin5_4.*t3)./6.0+(Alin5_4.*t4)./2.4e+1+Alin5_4.*T_k,(Alin6_4.*t2)./2.0+(Alin6_4.*t3)./6.0+(Alin6_4.*t4)./2.4e+1+Alin6_4.*T_k,(Alin7_4.*t2)./2.0+(Alin7_4.*t3)./6.0+(Alin7_4.*t4)./2.4e+1+Alin7_4.*T_k,(Alin8_4.*t2)./2.0+(Alin8_4.*t3)./6.0+(Alin8_4.*t4)./2.4e+1+Alin8_4.*T_k,(Alin9_4.*t2)./2.0+(Alin9_4.*t3)./6.0+(Alin9_4.*t4)./2.4e+1+Alin9_4.*T_k,(Alin10_4.*t2)./2.0+(Alin10_4.*t3)./6.0+(Alin10_4.*t4)./2.4e+1+Alin10_4.*T_k];
mt6 = [(Alin1_5.*t2)./2.0+(Alin1_5.*t3)./6.0+(Alin1_5.*t4)./2.4e+1+Alin1_5.*T_k,(Alin2_5.*t2)./2.0+(Alin2_5.*t3)./6.0+(Alin2_5.*t4)./2.4e+1+Alin2_5.*T_k,(Alin3_5.*t2)./2.0+(Alin3_5.*t3)./6.0+(Alin3_5.*t4)./2.4e+1+Alin3_5.*T_k,(Alin4_5.*t2)./2.0+(Alin4_5.*t3)./6.0+(Alin4_5.*t4)./2.4e+1+Alin4_5.*T_k,(Alin5_5.*t2)./2.0+(Alin5_5.*t3)./6.0+(Alin5_5.*t4)./2.4e+1+Alin5_5.*T_k+1.0,(Alin6_5.*t2)./2.0+(Alin6_5.*t3)./6.0+(Alin6_5.*t4)./2.4e+1+Alin6_5.*T_k,(Alin7_5.*t2)./2.0+(Alin7_5.*t3)./6.0+(Alin7_5.*t4)./2.4e+1+Alin7_5.*T_k,(Alin8_5.*t2)./2.0+(Alin8_5.*t3)./6.0+(Alin8_5.*t4)./2.4e+1+Alin8_5.*T_k];
mt7 = [(Alin9_5.*t2)./2.0+(Alin9_5.*t3)./6.0+(Alin9_5.*t4)./2.4e+1+Alin9_5.*T_k,(Alin10_5.*t2)./2.0+(Alin10_5.*t3)./6.0+(Alin10_5.*t4)./2.4e+1+Alin10_5.*T_k,(Alin1_6.*t2)./2.0+(Alin1_6.*t3)./6.0+(Alin1_6.*t4)./2.4e+1+Alin1_6.*T_k,(Alin2_6.*t2)./2.0+(Alin2_6.*t3)./6.0+(Alin2_6.*t4)./2.4e+1+Alin2_6.*T_k,(Alin3_6.*t2)./2.0+(Alin3_6.*t3)./6.0+(Alin3_6.*t4)./2.4e+1+Alin3_6.*T_k,(Alin4_6.*t2)./2.0+(Alin4_6.*t3)./6.0+(Alin4_6.*t4)./2.4e+1+Alin4_6.*T_k,(Alin5_6.*t2)./2.0+(Alin5_6.*t3)./6.0+(Alin5_6.*t4)./2.4e+1+Alin5_6.*T_k,(Alin6_6.*t2)./2.0+(Alin6_6.*t3)./6.0+(Alin6_6.*t4)./2.4e+1+Alin6_6.*T_k+1.0];
mt8 = [(Alin7_6.*t2)./2.0+(Alin7_6.*t3)./6.0+(Alin7_6.*t4)./2.4e+1+Alin7_6.*T_k,(Alin8_6.*t2)./2.0+(Alin8_6.*t3)./6.0+(Alin8_6.*t4)./2.4e+1+Alin8_6.*T_k,(Alin9_6.*t2)./2.0+(Alin9_6.*t3)./6.0+(Alin9_6.*t4)./2.4e+1+Alin9_6.*T_k,(Alin10_6.*t2)./2.0+(Alin10_6.*t3)./6.0+(Alin10_6.*t4)./2.4e+1+Alin10_6.*T_k,(Alin1_7.*t2)./2.0+(Alin1_7.*t3)./6.0+(Alin1_7.*t4)./2.4e+1+Alin1_7.*T_k,(Alin2_7.*t2)./2.0+(Alin2_7.*t3)./6.0+(Alin2_7.*t4)./2.4e+1+Alin2_7.*T_k,(Alin3_7.*t2)./2.0+(Alin3_7.*t3)./6.0+(Alin3_7.*t4)./2.4e+1+Alin3_7.*T_k,(Alin4_7.*t2)./2.0+(Alin4_7.*t3)./6.0+(Alin4_7.*t4)./2.4e+1+Alin4_7.*T_k];
mt9 = [(Alin5_7.*t2)./2.0+(Alin5_7.*t3)./6.0+(Alin5_7.*t4)./2.4e+1+Alin5_7.*T_k,(Alin6_7.*t2)./2.0+(Alin6_7.*t3)./6.0+(Alin6_7.*t4)./2.4e+1+Alin6_7.*T_k,(Alin7_7.*t2)./2.0+(Alin7_7.*t3)./6.0+(Alin7_7.*t4)./2.4e+1+Alin7_7.*T_k+1.0,(Alin8_7.*t2)./2.0+(Alin8_7.*t3)./6.0+(Alin8_7.*t4)./2.4e+1+Alin8_7.*T_k,(Alin9_7.*t2)./2.0+(Alin9_7.*t3)./6.0+(Alin9_7.*t4)./2.4e+1+Alin9_7.*T_k,(Alin10_7.*t2)./2.0+(Alin10_7.*t3)./6.0+(Alin10_7.*t4)./2.4e+1+Alin10_7.*T_k,(Alin1_8.*t2)./2.0+(Alin1_8.*t3)./6.0+(Alin1_8.*t4)./2.4e+1+Alin1_8.*T_k,(Alin2_8.*t2)./2.0+(Alin2_8.*t3)./6.0+(Alin2_8.*t4)./2.4e+1+Alin2_8.*T_k];
mt10 = [(Alin3_8.*t2)./2.0+(Alin3_8.*t3)./6.0+(Alin3_8.*t4)./2.4e+1+Alin3_8.*T_k,(Alin4_8.*t2)./2.0+(Alin4_8.*t3)./6.0+(Alin4_8.*t4)./2.4e+1+Alin4_8.*T_k,(Alin5_8.*t2)./2.0+(Alin5_8.*t3)./6.0+(Alin5_8.*t4)./2.4e+1+Alin5_8.*T_k,(Alin6_8.*t2)./2.0+(Alin6_8.*t3)./6.0+(Alin6_8.*t4)./2.4e+1+Alin6_8.*T_k,(Alin7_8.*t2)./2.0+(Alin7_8.*t3)./6.0+(Alin7_8.*t4)./2.4e+1+Alin7_8.*T_k,(Alin8_8.*t2)./2.0+(Alin8_8.*t3)./6.0+(Alin8_8.*t4)./2.4e+1+Alin8_8.*T_k+1.0,(Alin9_8.*t2)./2.0+(Alin9_8.*t3)./6.0+(Alin9_8.*t4)./2.4e+1+Alin9_8.*T_k,(Alin10_8.*t2)./2.0+(Alin10_8.*t3)./6.0+(Alin10_8.*t4)./2.4e+1+Alin10_8.*T_k];
mt11 = [(Alin1_9.*t2)./2.0+(Alin1_9.*t3)./6.0+(Alin1_9.*t4)./2.4e+1+Alin1_9.*T_k,(Alin2_9.*t2)./2.0+(Alin2_9.*t3)./6.0+(Alin2_9.*t4)./2.4e+1+Alin2_9.*T_k,(Alin3_9.*t2)./2.0+(Alin3_9.*t3)./6.0+(Alin3_9.*t4)./2.4e+1+Alin3_9.*T_k,(Alin4_9.*t2)./2.0+(Alin4_9.*t3)./6.0+(Alin4_9.*t4)./2.4e+1+Alin4_9.*T_k,(Alin5_9.*t2)./2.0+(Alin5_9.*t3)./6.0+(Alin5_9.*t4)./2.4e+1+Alin5_9.*T_k,(Alin6_9.*t2)./2.0+(Alin6_9.*t3)./6.0+(Alin6_9.*t4)./2.4e+1+Alin6_9.*T_k,(Alin7_9.*t2)./2.0+(Alin7_9.*t3)./6.0+(Alin7_9.*t4)./2.4e+1+Alin7_9.*T_k,(Alin8_9.*t2)./2.0+(Alin8_9.*t3)./6.0+(Alin8_9.*t4)./2.4e+1+Alin8_9.*T_k];
mt12 = [(Alin9_9.*t2)./2.0+(Alin9_9.*t3)./6.0+(Alin9_9.*t4)./2.4e+1+Alin9_9.*T_k+1.0,(Alin10_9.*t2)./2.0+(Alin10_9.*t3)./6.0+(Alin10_9.*t4)./2.4e+1+Alin10_9.*T_k,(Alin1_10.*t2)./2.0+(Alin1_10.*t3)./6.0+(Alin1_10.*t4)./2.4e+1+Alin1_10.*T_k,(Alin2_10.*t2)./2.0+(Alin2_10.*t3)./6.0+(Alin2_10.*t4)./2.4e+1+Alin2_10.*T_k,(Alin3_10.*t2)./2.0+(Alin3_10.*t3)./6.0+(Alin3_10.*t4)./2.4e+1+Alin3_10.*T_k,(Alin4_10.*t2)./2.0+(Alin4_10.*t3)./6.0+(Alin4_10.*t4)./2.4e+1+Alin4_10.*T_k,(Alin5_10.*t2)./2.0+(Alin5_10.*t3)./6.0+(Alin5_10.*t4)./2.4e+1+Alin5_10.*T_k,(Alin6_10.*t2)./2.0+(Alin6_10.*t3)./6.0+(Alin6_10.*t4)./2.4e+1+Alin6_10.*T_k];
mt13 = [(Alin7_10.*t2)./2.0+(Alin7_10.*t3)./6.0+(Alin7_10.*t4)./2.4e+1+Alin7_10.*T_k,(Alin8_10.*t2)./2.0+(Alin8_10.*t3)./6.0+(Alin8_10.*t4)./2.4e+1+Alin8_10.*T_k,(Alin9_10.*t2)./2.0+(Alin9_10.*t3)./6.0+(Alin9_10.*t4)./2.4e+1+Alin9_10.*T_k,(Alin10_10.*t2)./2.0+(Alin10_10.*t3)./6.0+(Alin10_10.*t4)./2.4e+1+Alin10_10.*T_k+1.0];
out1 = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11,mt12,mt13],10,10);
end
