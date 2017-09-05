m = 60;
for i=1:m
for j=1:i
    a = VCoosek(1:i,j);
    s = ['nchoosek', num2str(i),'_',num2str(j),'.mat']
    save( s, 'a' );
end
end