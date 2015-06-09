filename = 'log_level_3.txt';
fid =  fopen(filename, 'r');
text = textscan(fid, '%s', 'Delimiter','\n');
fid = fclose(fid);

length_text = length(text{1});
for i = 1:(length_text - 1)
 if (text{1}{i}(1) == 'W') 
  start = i + 1;
 end
end

index = 1;
num = zeros(1, length_text - start);
for j = start:(length_text - 1)
  temp = textscan(text{1}{j}, '%s %d %s %d');
  num(index) = temp{4};
  index = index + 1;
end

disp(filename)
mu = mean(num)
sigma = std(num)
var = sigma^2