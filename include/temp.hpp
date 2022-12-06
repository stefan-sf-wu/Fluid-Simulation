        #pragma omp parallel for
        for (int i = 0; i < k_num_particle; i++)
        {
            collision::result ret = collision::detect_collision(particles.position.at(i), particles.next_position.at(i), 
                                                                particles.velocity.at(i), particles.next_velocity.at(i));
            if (ret != collision::null_result) 
            {
                particles.position.at(i) = particles.next_position.at(i);
                particles.next_position.at(i) = ret.new_pos;
                particles.next_velocity.at(i) = ret.new_vel;
            }
            else 
            {
                std::cout << "not col\n";
                particles.position.at(i) = particles.next_position.at(i);
            }
        }