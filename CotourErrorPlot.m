function []=CotourErrorPlot(TrueTipContourError,TrueOrienContourError,Proposed,Yang)

figure(6)
plot(Proposed(1,:),TrueTipContourError(:,4)*1e3,'b','Linewidth',2)
hold on
plot(Proposed(1,:),Proposed(2,:)*1e3,'r--','Linewidth',2);
xlabel('Time [sec]')
ylabel('Contour Error [micron]');
title('Comparision of True and Proposed Estimation Tool Tip Contour Errors');
hold off;

figure(7)
plot(Yang(1,:),TrueTipContourError(:,4)*1e3,'b','Linewidth',2)
hold on
plot(Yang(1,:),Yang(2,:)*1e3,'r--','Linewidth',2);
xlabel('Time [sec]')
ylabel('Contour Error [micron]');
title('Comparision of True and Yang Estimation Tool Tip Contour Errors');
hold off;

figure(8)
plot(Proposed(1,:),(TrueTipContourError(:,4)'-Proposed(2,:))*1e3,'b','Linewidth',2)
hold on
plot(Yang(1,:),(TrueTipContourError(:,4)'-Yang(2,:))*1e3,'r--','Linewidth',2);
xlabel('Time [sec]')
ylabel('Discrepency [micron]');
title('Discrepency of True and Estimation Tool Tip Contour Errors');
legend('Proposed Method','Yang Method');
hold off;

figure(9)
plot(Proposed(1,:),TrueOrienContourError*1e3,'b','Linewidth',2)
hold on
plot(Proposed(1,:),Proposed(3,:)*1e3,'r--','Linewidth',2);
xlabel('Time [sec]')
ylabel('Contour Error [milliradians]');
title('Comparision of True and Proposed Estimation Tool Orientation Contour Errors');
hold off;

figure(10)
plot(Yang(1,:),TrueOrienContourError*1e3,'b','Linewidth',2)
hold on
plot(Yang(1,:),Yang(3,:)*1e3,'r--','Linewidth',2);
xlabel('Time [sec]')
ylabel('Contour Error [milliradians]');
title('Comparision of True and Yang Estimation Tool Orientation Contour Errors');
hold off;

figure(11)
plot(Proposed(1,:),(TrueOrienContourError'-Proposed(3,:))*1e6,'b','Linewidth',2)
hold on
plot(Yang(1,:),(TrueOrienContourError'-Yang(3,:))*1e6,'r--','Linewidth',2)
xlabel('Time [sec]')
ylabel('Discrepency [milliradians]');
title('Discrepency of True and Estimation Tool Orientation Contour Errors');
legend('Proposed Method','Yang Method');
hold off;