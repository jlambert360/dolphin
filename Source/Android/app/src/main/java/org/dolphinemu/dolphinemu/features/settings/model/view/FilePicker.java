package org.dolphinemu.dolphinemu.features.settings.model.view;

import org.dolphinemu.dolphinemu.features.settings.model.Setting;
import org.dolphinemu.dolphinemu.features.settings.model.StringSetting;
import org.dolphinemu.dolphinemu.features.settings.utils.SettingsFile;

import java.io.File;

public final class FilePicker extends SettingsItem
{
  private String mFile;
  private String mDefaultValue;
  private int mRequestType;

  public FilePicker(String file, String key, String section, int titleId, int descriptionId,
          String defaultVault, int requestType, Setting setting)
  {
    super(key, section, setting, titleId, descriptionId);
    mFile = file;
    mDefaultValue = defaultVault;
    mRequestType = requestType;
  }

  public File getFile()
  {
    return SettingsFile.getSettingsFile(mFile);
  }

  public String getSelectedValue()
  {
    if (getSetting() == null || !(getSetting() instanceof StringSetting))
    {
      return mDefaultValue;
    }
    else
    {
      StringSetting setting = (StringSetting) getSetting();
      return setting.getValue();
    }
  }

  public int getRequestType()
  {
    return mRequestType;
  }

  @Override
  public int getType()
  {
    return TYPE_FILE_PICKER;
  }
}
