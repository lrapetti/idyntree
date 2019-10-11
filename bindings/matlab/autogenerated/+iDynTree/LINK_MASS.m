function v = LINK_MASS()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 35);
  end
  v = vInitialized;
end
