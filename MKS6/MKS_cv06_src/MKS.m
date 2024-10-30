a = csvread("ntc.csv");

r = a(:,2);
ad = (r)./(r+10)*1024;
t = a(:,1);

p = polyfit(ad, t, 10);
stem(ad,t)

ad2 = 0:1024;

t2 = round(polyval(p, ad2), 1);
hold on, plot(ad2, t2, 'r');

dlmwrite('data.dlm', t2*10, ',');
