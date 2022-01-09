class SettingsController < ApplicationController
  def index
  end

  def update
    error_count = 0
    params[:settings].each do |name, value|
      if value.blank?
        error_count += 1
        next
      end

      if setting = Setting.find_by(name: name)
        setting.update!(value: value)
      else
        Setting.create!(name: name, value: value)
      end
    end

    if error_count > 0
      flash[:error] = "No settings can be blank, please try again"
    else
      flash[:notice] = "Settings successfully updated"
    end

    render :index
  end
end
