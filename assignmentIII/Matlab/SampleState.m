function s  = SampleState()
  global params;
  s(1) = RandomReal(params.xmin, params.xmax);
  s(2) = RandomReal(params.ymin, params.ymax); 
end

