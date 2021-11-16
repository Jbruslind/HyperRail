Rails.application.routes.draw do
  resource :session, only: [:new, :create, :destroy]
  resources :programs

  get '/dashboard', to: 'dashboard#index', as: :dashboard
  root to: 'dashboard#index'
end
