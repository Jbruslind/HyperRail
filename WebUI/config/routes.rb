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

  match 'download_zip', to: 'programs#download_zip', as: 'download_zip', via: :get

  get '/camera_images/*path', format: false, to: 'camera_images#show', as: :camera_image

  get '/settings', to: 'settings#index', as: :settings
  post '/settings', to: 'settings#update', as: :update_settings

  get '/dashboard', to: 'dashboard#index', as: :dashboard
  root to: 'dashboard#index'
end
