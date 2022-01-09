class CreateProgramRuns < ActiveRecord::Migration[6.1]
  def change
    create_table :program_runs do |t|
      t.integer :program_id, null: false
      t.timestamp :started_at, null: false
      t.timestamp :finished_at
    end

    add_foreign_key :program_runs, :programs, on_delete: :cascade
    add_index :program_runs, :program_id
  end
end
