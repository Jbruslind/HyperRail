class SessionsController < ApplicationController
  skip_before_action :login_required

  def new
  end

  def create
    if params[:password] == current_password
      cookies.encrypted.signed[:session] = JSON.generate({logged_in_at: Time.now.utc, password: params[:password]})
      redirect_to :dashboard
    else
      cookies.delete(:session)
      flash[:error] = "Password incorrect, please try again"
      redirect_to :new_session
    end
  end

  def destroy
    cookies.delete(:session)
    flash[:notice] = "You have logged out"
    redirect_to :new_session
  end
end
