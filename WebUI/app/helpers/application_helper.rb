module ApplicationHelper
  def nav_active_for(str)
    params[:controller].to_s =~ /#{Regexp.escape(str)}/i ? 'active' : ''
  end

  def bootstrap_class_for(flash_type)
    case flash_type.to_sym
      when :success
        "alert-success"
      when :error
        "alert-danger"
      when :alert
        "alert-warning"
      when :notice
        "alert-info"
      else
        flash_type.to_s
    end
  end
end
