ÖDEV 3
#RobotSüpürge

Bir robot süpürgenin temizleyeceği alanı turtlesim simülasyonuyla yaptık. Bu simülasyonda turtlesim'e nerelere gitmesi gerektiği, yönünü ne tarafa çevirmesi gerektiği, kaç birim ilerlemesi gerektiği gibi bilgileri göndererek robot süpürgenin her alanı kaplayacak şekilde temizlik yapmasını sağladık.


Kullanım
	Önce ROS MASTER'ı çalıştırın. Bunun için terminale aşağıdaki komutu yazınız;
		roscore
	Daha sonra projeyi çalıştırmak için turtlesim simülasyonunu açmamız gerek. Bunun içinde yeni bir terminal açarak aşağıdaki komutu giriniz;
		rosrun turtlesim turtlesim_node
	Sonrasında süpürgemize temizleyeceği alanı anlattığımız kodu çalıştıracağız. Bunun için yeni terminal açıp şu komutları giriniz;
		rosrun ros_tutorials turtlesim_cleaner.py

Robot süpürgenin etkili bir şekilde alanı kaplaması için bir zigzag hareket algoritması geliştirilmiştir. Bu algoritma, robotun belirli bir mesafe hareket ettikten sonra yön değiştirmesini sağlayarak temizleme işlemini gerçekleştirir.

Hareket Kontrolü: Robot, belirli bir mesafeyi kat ettikten sonra yön değiştirerek temizleme alanının her bir noktasına ulaşmayı hedefler.
Zigzag Hareket: Robot, ileri doğru hareket ettikten sonra sağa veya sola dönerek temizleme alanını kaplar. Bu hareketler, robotun her bölgeyi eşit şekilde temizlemesini sağlar.

ARKA PLAN
Bir dosya oluşturduk ve dosyanın içine python kodlarımız bulunan dosyaları attık.
Attıktan sonra içlerini düzenleyerek istenilen Robot Süpürgeyi yapmış olduk.
