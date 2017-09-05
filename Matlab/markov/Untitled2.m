m = 60;
for i=28:m
for j=1:i
    if i==28 && j <= 12
        continue
    else
        a = nchoosek(1:i,j);
        s = ['nchoosek', num2str(i),'_',num2str(j),'.mat']
        save( s, 'a' );
    end
end
end