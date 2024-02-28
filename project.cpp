#include <SFML/Window.hpp>

int main()
{
    sf::Window window;
    window.create(sf::VideoMode(1080, 720), "My window", sf::Style::Close | sf::Style::Resize);
    window.setVerticalSyncEnabled(true); 
    window.setPosition(sf::Vector2i(400, 200));
    // программа будет запущена до тех пор, пока окно открыто
    while (window.isOpen())
    {
        // проверка всех событий окна, произошедших с последней итерации цикла
        sf::Event event;
        while (window.pollEvent(event))
        {
            // пользователь попытался закрыть окно: мы закрываем окно
            if (event.type == sf::Event::Closed)
                window.close();
        }
    }

    return 0;
}