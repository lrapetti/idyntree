function v = COM_ACCELEROMETER_SENSOR()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = iDynTreeMEX(0, 32);
  end
  v = vInitialized;
end
