clc;clear all;close;
    
    [angle1,x1,y1]=textread('control_pos_m_sim.txt','%f%f%f','headerlines',0);
 %   angle1=angle1+pi/2;
%     x1=x1*100;
%     y1=y1*100;
    y1=y1+1.45;
    x1 = x1+.1;
   
   figure(1); 
   halfL = 0.41/2; halfW =0.41/2;     %%%   original robot posture (corner)
   cornerR = [halfL -halfL -halfL halfL halfL;
   halfW halfW -halfW -halfW halfW];
   arrowR = [1.5*halfL 0]';  % robot orientation arrow

    for i=0:1:size(x1,1)-1
       x2(1+ i/1)=x1(i+1)-0;
        y2(1+ i/1)=y1(i+1)-0;
    end
    plot(x2,y2,'r','LineWidth',2);hold on;
%     plot(x1,y1,'r','LineWidth',1);hold on;
    
   for i=1:50000:size(x1,1)
       %%if((((x(i+1)-x(i))^2+y(i+1)-y(i))^2) < 6) 
        
        
           angle1(i) = wrapToPi( angle1(i) );
           rotationM = [cos(angle1(i))  -sin(angle1(i)) ; sin(angle1(i))  cos(angle1(i))];
           rotated_cornerR = rotationM * cornerR + [x1(i)-0 0;0 y1(i)-0]*ones(size(cornerR));
           rotated_arrowR = rotationM * arrowR + [x1(i)-0 y1(i)-0]';    
           plot(rotated_cornerR(1,:), rotated_cornerR(2,:),'g-','LineWidth',2.5); hold on;
           plot([x1(i)-0 rotated_arrowR(1)],[y1(i)-0 rotated_arrowR(2)],'b-','LineWidth',1.5); hold on;
           if i==1;
%                plot(rotated_cornerR(1,:), rotated_cornerR(2,:),'r-','LineWidth',2.5); hold on;
%                plot([x1(i)-0 rotated_arrowR(1)],[y1(i)-0 rotated_arrowR(2)],'r-','LineWidth',1.5); hold on;
           end
      %% end
   end
  plot(rotated_cornerR(1,:), rotated_cornerR(2,:),'--black','LineWidth',2.5); hold on;
  plot([x1(i)-0 rotated_arrowR(1)],[y1(i)-0 rotated_arrowR(2)],'--black','LineWidth',1.5); hold on;
  
title('\fontsize{18} \fontname{Times New Roman} trajectory')
xlabel('\fontsize{18} \fontname{Times New Roman} m');
ylabel('\fontsize{18} \fontname{Times New Roman} m');
  
%axis([-1.5,1.5,-2,1.5]);
grid on;axis equal;hold on;
set(gca,'FontName','Times New Roman','FontSize',18)

   