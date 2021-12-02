Rails.application.routes.draw do
  resources :analyses
  resource :session, only: [:new, :create, :destroy]
  resources :programs do
    collection do
      post 'default'
    end
  end
  resources :analyses

  get '/settings', to: 'settings#index', as: :settings
  post '/settings', to: 'settings#update', as: :update_settings

  get '/dashboard', to: 'dashboard#index', as: :dashboard
  root to: 'dashboard#index'
end
