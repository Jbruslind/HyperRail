class CreateSettings < ActiveRecord::Migration[6.1]
  def change
    create_table :settings do |t|
      t.string :name, null: false, unique: true
      t.string :value
      t.timestamps
    end

    add_index :settings, :name, unique: true
  end
end
