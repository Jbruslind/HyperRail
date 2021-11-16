class ApplicationController < ActionController::Base
  before_action :login_required

  private

  def logged_in?
    @logged_in_at.present?
  end
  helper_method :logged_in?

  def login_required
    session = JSON.parse(cookies.encrypted.signed[:session] || '{}')
    if session['password'] != current_password
      cookies.delete(:session)
      flash[:error] = "Please log in"
      redirect_to :new_session
    else
      @logged_in_at = session['logged_in_at']
    end
  end

  def current_password
    Setting.fetch('password', 'default-password')
  end
end
