% Vdc_R = 821.576kV Vdc_I = 796.633kV Idc = 5kA Vrms_R = 0.99pu*500kV 
% alpha = 0.485/pi*180 = 27.7885° Xt = 2.7747!确定


clear all;
P_wf1_start = 589.54;
P_wf2_start = 589.54;
Tr_K = 1.05;
Cov_N = 2;
Xt = 2.7747;
UdcI_start = 800;
Rdc = 5;
UdcR_L = 400;
Iac_max = 1.6;
% M = 466.67;
sub_N = 252;
MMC_N = 3;
C0 = 15000*10^-6;
Ceq = 6/sub_N*C0*MMC_N;
K_Idc = 48.48;
% K_WF = 372;
K_WF = 0;

syms x y
syms cos_alpha 
j = 1;
for UacR_pu = 1:-0.02:0.2
    cos_alpha = vpasolve((Cov_N*(3*sqrt(2)/pi*UacR_pu*180.3*Tr_K*x-3/pi*Xt*5)+UdcR_L-UdcI_start)/5 == 5,x);
%   判断LCC的工作模式：定关断角还是定直流电流，计算LCC的触发角
    if cos_alpha > 0.9962
        alpha(j) = acos(0.9962);
        Idc(j) = vpasolve((Cov_N*(3*sqrt(2)/pi*UacR_pu*180.3*Tr_K*0.9962-3/pi*Xt*y)+UdcR_L-UdcI_start)/5 == y,y);
%       判断是否触发受端MMC下垂控制
        if Idc(j) < 5
            Idc(j) = vpasolve((Cov_N*(3*sqrt(2)/pi*UacR_pu*180.3*Tr_K*0.9962-3/pi*Xt*y)+UdcR_L-(UdcI_start-K_Idc*(5-y)))/5 == y,y);
            UdcI(j) = UdcI_start-K_Idc*(5-Idc(j));
%           判断MMC受端下垂是否到达限幅
            if UdcI(j) < 640
                UdcI(j) = 640;
                Idc(j) = vpasolve((Cov_N*(3*sqrt(2)/pi*UacR_pu*180.3*Tr_K*0.9962-3/pi*Xt*y)+UdcR_L-640)/5 == y,y);
            else
                Idc(j) = Idc(j);
                UdcI(j) = UdcI(j);
            end
%          判断风机功率减载是否触发
            if Idc(j) < 1.6
                P_wf2(j) = P_wf2_start-K_WF*(1.6-Idc(j));
                P_wf1(j) = P_wf1_start-K_WF*(1.6-Idc(j));
                if P_wf1 (j) < 300
                    P_wf2(j) = 300;
                    P_wf1(j) = 300;
                else
                    P_wf2(j) =  P_wf2(j);
                    P_wf1(j) =  P_wf1(j); 
                end
            else
                P_wf2(j) = P_wf2_start;
                P_wf1(j) = P_wf1_start;
            end
%          判断直流电流是否大于0     
            if Idc(j) < 0
                Idc(j) = 0;
            else
                Idc(j) = Idc(j);
            end
        else
            Idc(j) = 5;
            UdcI(j) = 800;
        end
    else
        alpha(j) = acos(cos_alpha);
        Idc(j) = 5;
        UdcI(j) = 800;
        P_wf2(j) = P_wf2_start;
        P_wf1(j) = P_wf1_start;
    end
    UdcR_H(j) = 2*(3*sqrt(2)/pi*UacR_pu*180.3*1.05*cos(alpha(j))-3/pi*Xt*Idc(j));
    UdcR(j) = UdcR_H(j)+UdcR_L;
    PdcR_L(j) = UdcR_L*Idc(j);
    PdcR_H(j) = UdcR_H(j)*Idc(j);
    PdcR(j) = UdcR(j)*Idc(j);
    delta_P(j) = PdcR(j)-P_wf2(j)-P_wf1(j);
    if delta_P(j) < 0
        surplus_P(j) = UacR_pu*500*Iac_max+delta_P(j);
        if surplus_P(j) > 0
            surplus_P(j) = 0;
        else
            surplus_P(j) = -surplus_P(j);
        end
    else
        surplus_P(j) = 0;
    end
    i = 1;
    for fault_time = 0.1:0.05:1
%       当盈余有功功率存在时，计算直流电压上升
        if surplus_P(j) > 0
            UdcR_L_rt = 400;
            Idc_rt(j,i) = Idc(j);
            UdcR_rt(j,i) = UdcR(j);
            P_wf1_rt(j,i) = P_wf1(j);
            P_wf2_rt(j,i) = P_wf2(j);
            for t = 0.01:0.005:fault_time
                delta_P_rt(j,i) = P_wf1_rt(j,i)+P_wf2_rt(j,i)-sqrt(3)*UacR_pu*500*Iac_max-Idc_rt(j,i)*UdcR_L_rt;
                
                %当没有有功盈余时，低压阀组的直流电压会停止上升
                if delta_P_rt(j,i) < 0 

                    break
                end

%               参考文献"柔性直流电网直流过电压分析及控制策略研究"-电网技术，直流过电压计算方式
                UdcR_L_rt = abs(sqrt(delta_P_rt(j,i)*0.005/Ceq+UdcR_L_rt^2));
                Idc_rt(j,i) = vpasolve((Cov_N*(3*sqrt(2)/pi*UacR_pu*180.3*Tr_K*0.9962-3/pi*Xt*y)+UdcR_L_rt-(UdcI_start-K_Idc*(5-y)))/5 == y,y);

                if Idc_rt(j,i) < 1.6
                   P_wf1_rt(j,i) = P_wf1_start-K_Idc*(5-Idc_rt(j,i));
                   P_wf2_rt(j,i) = P_wf1_start-K_Idc*(5-Idc_rt(j,i));
                   if P_wf2_rt(j,i) < 300
                       P_wf2_rt(j,i) = 300;
                       P_wf1_rt(j,i) = 300;
                   end
                end
                if Idc_rt(j,i) < 0
                    Idc_rt(j,i) = 0;
                else
                    Idc_rt(j,i) = Idc_rt(j,i);
                end
            end
            UdcR_L_fin(j,i) = UdcR_L_rt;
        else
            UdcR_L_fin(j,i) = 400;
        end
        i = i+1;
    end
    j = j+1;
end

VacR = [1:-0.02:0.2];
fault_duration = [0:0.05:1];
[x y]=meshgrid(fault_duration,VacR);
mesh(x,y,UdcR_L_fin);
% xlabel('fault duration(s)');
% ylabel('AC voltage (p.u.)');
% zlabel('VSC overvoltage(kV)');


% figure (1)
% plot(alpha)
% hold on
% 
% figure (2)
% plot(Idc)
% hold on
% 
% figure (3)
% plot(surplus_P)
% hold on
% 
% figure (4)
% plot(delta_P)
% hold on
