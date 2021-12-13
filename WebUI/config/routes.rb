Rails.application.routes.draw do
  resources :analyses do
    member do
      get :aggregate
      get :individual
    end
  end

  resource :session, only: [:new, :create, :destroy]
  resources :programs do
    collection do
      post 'default'
    end
  end

  get '/camera_images/*path', format: false, to: 'camera_images#show', as: :camera_image

  get '/settings', to: 'settings#index', as: :settings
  post '/settings', to: 'settings#update', as: :update_settings

  get '/dashboard', to: 'dashboard#index', as: :dashboard
  root to: 'dashboard#index'
end
