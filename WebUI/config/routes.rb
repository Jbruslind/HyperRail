Rails.application.routes.draw do
  resources :analyses
  resource :session, only: [:new, :create, :destroy]
  resources :programs
  resources :analyses

  get '/dashboard', to: 'dashboard#index', as: :dashboard
  root to: 'dashboard#index'
end
