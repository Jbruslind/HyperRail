class CreatePrograms < ActiveRecord::Migration[6.1]
  def change
    create_table :programs do |t|
      t.string :name, null: false
      t.timestamp :last_run_at
      t.timestamp :archived_at
    end
  end
end
